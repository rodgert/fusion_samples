/*
    Copyright (c) 2014 Contributors as noted in the AUTHORS file

    Distributed under the Boost Software License, Version 1.0. (See accompanying
    file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/


#include "network.hpp"

#include <boost/utility/string_ref.hpp>
#include <boost/optional.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/fusion/include/define_struct.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/is_sequence.hpp>
#include <boost/system/system_error.hpp>
#include <boost/system/error_code.hpp>
#include <boost/iterator/iterator_facade.hpp>

#include <cstddef>
#include <type_traits>
#include <array>
#include <bitset>

#include <iostream>

namespace asio = boost::asio;
namespace fusion = boost::fusion;

namespace example {
    template<typename T, size_t N = CHAR_BIT * sizeof(T)>
    struct optional_field_set {
        using value_type = T;
        using bits_type = std::bitset<N>;
    };

    template<typename T, size_t N>
    struct optional_field : boost::optional<T> {
        constexpr static const size_t bit = N;
        optional_field() = default;
        optional_field(T v) : boost::optional<T>(std::move(v)) { }
    };

    using opt_fields = optional_field_set<uint16_t>;

    template<class T>
    struct lazy {
        asio::const_buffer buf_;
        boost::optional<T> val_;

        lazy()
            : val_(T())
        { }

        lazy(T val)
            : val_(std::move(val))
        { }

        lazy(asio::const_buffer const& buf);

        T& get();
        T const& get() const { return *val_; }

        bool has_value() const { return !!val_; }
        size_t buffer_size() const { return asio::buffer_size(buf_); }
    };

    template<class T>
    struct lazy_const_iterator;

    template<class T>
    struct lazy_range {
        uint16_t count_;
        asio::const_buffer buf_;

        lazy_range(uint16_t count = 0, asio::const_buffer = asio::const_buffer());

        lazy_const_iterator<T> begin() const;
        lazy_const_iterator<T> end() const;

        size_t buffer_size() const { return asio::buffer_size(buf_); }
    };
}

namespace detail {
    namespace bs = boost::system;

bs::system_error bad_message() {
    return bs::system_error(bs::errc::make_error_code(bs::errc::bad_message));
}

struct reader {
    mutable asio::const_buffer buf_;
    mutable boost::optional<example::opt_fields::bits_type> opts_;

    explicit reader(asio::const_buffer buf)
        : buf_(std::move(buf))
    { }

    template<class T>
    auto operator()(T & val) const ->
        typename std::enable_if<std::is_integral<T>::value>::type {
        val = net::ntoh(*asio::buffer_cast<T const*>(buf_));
        buf_ = buf_ + sizeof(T);
    }

    template<class T>
    auto operator()(T & val) const ->
        typename std::enable_if<std::is_enum<T>::value>::type {
        typename std::underlying_type<T>::type v;
        (*this)(v);
        val = static_cast<T>(v);
    }

    template<class T, T v>
    void operator()(std::integral_constant<T, v>) const {
        typedef std::integral_constant<T, v> type;
        typename type::value_type val;
        (*this)(val);
        if (val != type::value)
            throw bad_message();
    }

    void operator()(std::string& val) const {
        uint16_t length = 0;
        (*this)(length);
        val = std::string(asio::buffer_cast<char const*>(buf_), length);
        buf_ = buf_ + length;
    }

    void operator()(boost::string_ref& val) const {
        uint16_t length = 0;
        (*this)(length);
        val = boost::string_ref(asio::buffer_cast<char const*>(buf_), length);
        buf_ = buf_ + length;
    }

    void operator()(boost::posix_time::ptime& val) const {
        std::string iso_str;
        (*this)(iso_str);
        val = boost::posix_time::from_iso_string(iso_str);
    }

    template<class T>
    void operator()(std::vector<T>& val) const {
        uint16_t length = 0;
        (*this)(length);
        for (; length; --length) {
            T v;
            (*this)(v);
            val.emplace_back(v);
        }
    }

    template<class K, class V>
    void operator()(boost::container::flat_map<K,V>& val) const {
        uint16_t length = 0;
        (*this)(length);
        for (; length; --length) {
            K key;
            (*this)(key);
            V value;
            (*this)(value);
            val.emplace(key,value);
        }
    }

    void operator()(example::opt_fields&) const {
        example::opt_fields::value_type val;
        (*this)(val);
        opts_ = example::opt_fields::bits_type(val);
    }

    template<class T, size_t N>
    void operator()(example::optional_field<T, N> & val) const {
        if (!opts_)
            throw bad_message();
        if ((*opts_)[N]) {
            T v;
            (*this)(v);
            val = example::optional_field<T, N>(std::move(v));
        }
    }

    template<class T>
    auto operator()(T & val) const ->
    typename std::enable_if<boost::fusion::traits::is_sequence<T>::value>::type {
        boost::fusion::for_each(val, *this);
    }

    template<class T>
    void operator()(example::lazy<T> & val) const {
        val = example::lazy<T>(buf_);
        buf_ = buf_ + val.buffer_size();
    }

    template<class T>
    void operator()(example::lazy_range<T> & val) const {
        uint16_t length;
        (*this)(length);
        val = example::lazy_range<T>(length, buf_);
    }
};

struct writer {
    mutable asio::mutable_buffer buf_;
    mutable example::opt_fields::bits_type opts_;
    mutable example::opt_fields::value_type *optv_;

    explicit writer(asio::mutable_buffer buf)
    : buf_(std::move(buf))
    , optv_(nullptr)
    { }

    template<class T>
    auto operator()(T const& val) const ->
        typename std::enable_if<std::is_integral<T>::value>::type {
        T tmp = net::hton(val);
        asio::buffer_copy(buf_, asio::buffer(&tmp, sizeof(T)));
        buf_ = buf_ + sizeof(T);
    }

    template<class T>
    auto operator()(T const& val) const ->
        typename std::enable_if<std::is_enum<T>::value>::type {
        using utype = typename std::underlying_type<T>::type;
        (*this)(static_cast<utype>(val));
    }

    template<class T, T v>
    void operator()(std::integral_constant<T, v>) const {
        typedef std::integral_constant<T, v> type;
        (*this)(type::value);
    }

   void operator()(std::string const& val) const {
        (*this)(static_cast<uint16_t>(val.size()));
        asio::buffer_copy(buf_, asio::buffer(val));
        buf_ = buf_ + val.length();
    }

   void operator()(boost::string_ref const& val) const {
        (*this)(static_cast<uint16_t>(val.size()));
        asio::buffer_copy(buf_, asio::buffer(val.data(), val.size()));
        buf_ = buf_ + val.length();
    }

    void operator()(boost::posix_time::ptime const& val) const {
        auto iso_str = boost::posix_time::to_iso_string(val);
        (*this)(iso_str);
    }

    template<class T>
    void operator()(std::vector<T> const& vals) const {
        (*this)(static_cast<uint16_t>(vals.size()));
        for (auto&& val : vals)
            (*this)(val);
    }

    template<class K, class V>
    void operator()(boost::container::flat_map<K,V> const& vals) const {
        (*this)(static_cast<uint16_t>(vals.size()));
        for (auto&& val : vals) {
            (*this)(val.first);
            (*this)(val.second);
        }
    }

    void operator()(example::opt_fields) const {
        opts_.reset();
        optv_ = asio::buffer_cast<example::opt_fields::value_type*>(buf_);
        buf_ = buf_ + sizeof(example::opt_fields::value_type);
    }

    template<class T, size_t N>
    void operator()(example::optional_field<T, N> const& val) const {
        if (!optv_)
            throw bad_message();
        if (val) {
            opts_.set();
            *optv_ = static_cast<example::opt_fields::value_type>(opts_.to_ulong());
            (*this)(*val);
        }
    }

    template<class T>
    auto operator()(T const& val) const ->
    typename std::enable_if<boost::fusion::traits::is_sequence<T>::value>::type {
        boost::fusion::for_each(val, *this);
    }

    template<class T>
    void operator()(example::lazy<T> const& val) const {
        if (!val.has_value())
            throw bad_message();
        (*this)(val.get());
    }
};

struct sizer {
    mutable asio::const_buffer buf_;
    mutable boost::optional<example::opt_fields::bits_type> opts_;

    explicit sizer (asio::const_buffer buf)
        : buf_(std::move(buf))
    { }

    uint16_t get_length() const {
        reader r(buf_);
        uint16_t res;
        r(res);
        buf_ = r.buf_;
        return res;
    }

    template<class T>
    auto operator()(T const&) const ->
        typename std::enable_if<std::is_integral<T>::value>::type {
        buf_ = buf_ + sizeof(T);
    }

    template<class T>
    auto operator()(T const&) const ->
        typename std::enable_if<std::is_enum<T>::value>::type {
        typename std::underlying_type<T>::type v;
        (*this)(v);
    }

    template<class T, T v>
    void operator()(std::integral_constant<T, v>) const {
        typedef std::integral_constant<T, v> type;
        typename type::value_type val;
        (*this)(val);
    }

    void operator()(std::string const&) const { buf_ = buf_ + get_length(); }
    void operator()(boost::string_ref const&) const { buf_ = buf_ + get_length(); }
    void operator()(boost::posix_time::ptime const&) const { buf_ = buf_ + get_length(); }

    template<class T>
    void operator()(std::vector<T> const&) const {
        uint16_t length = get_length();
        for (; length; --length) {
            T v;
            (*this)(v);
        }
    }

    template<class K, class V>
    void operator()(boost::container::flat_map<K,V> const&) const {
        uint16_t length = get_length();
        for (; length; --length) {
            K key;
            (*this)(key);
            V value;
            (*this)(value);
        }
    }

    void operator()(example::opt_fields const&) const {
        reader r(buf_);
        example::opt_fields::value_type val;
        r(val);
        opts_ = example::opt_fields::bits_type(val);
        buf_ = r.buf_;
    }

    template<class T, size_t N>
    void operator()(example::optional_field<T, N> const&) const {
        if (!opts_)
            throw bad_message();
        if ((*opts_)[N]) {
            T v;
            (*this)(v);
        }
    }

    template<class T>
    auto operator()(T const& val) const ->
    typename std::enable_if<boost::fusion::traits::is_sequence<T>::value>::type {
        boost::fusion::for_each(val, *this);
    }
};
} // namespace detail

template<typename T>
std::pair<T, asio::const_buffer> read(asio::const_buffer b) {
    detail::reader r(std::move(b));
    T res;
    r(res);
    return std::make_pair(res, r.buf_);
}

template<typename T>
asio::mutable_buffer write(asio::mutable_buffer b, T const& val) {
    detail::writer w(std::move(b));
    w(val);
    return w.buf_;
}

template<class T>
size_t get_size(asio::const_buffer buf) {
    detail::sizer s(buf);
    T v;
    s(v);
    return asio::buffer_size(buf) - asio::buffer_size(s.buf_);
}

namespace example {
    template<class T>
    lazy<T>::lazy(asio::const_buffer const& buf)
            : buf_(asio::buffer_cast<void const*>(buf)
            , get_size<T>(buf))
    { }

    template<class T>
    T& lazy<T>::get() {
        if (!val_) {
            detail::reader r(buf_);
            T tmp;
            r(tmp);
            val_ = tmp;
        }
        return *val_;
    }

    template<class T>
    struct lazy_const_iterator
        : boost::iterator_facade<
            lazy_const_iterator<T>
          , T const
          , boost::forward_traversal_tag> {
        lazy_const_iterator(uint16_t count, asio::const_buffer buf)
            : count_(count)
            , buf_(buf)
        { }

    private:
        friend class boost::iterator_core_access;

        void increment() {
            T v;
            std::tie(v, buf_) = read<T>(buf_);
            ++count_;
        }

        bool equal(lazy_const_iterator<T> const& other) const
        {
            return count_ == other.count_;
        }

        T const& dereference() const { return *val_; }

        uint16_t count_;
        asio::const_buffer buf_;
        boost::optional<T> val_;
    };

    template<class T>
    lazy_range<T>::lazy_range(uint16_t count, asio::const_buffer buf)
        : count_(count)
        , buf_(buf)
    {
        for (; count != 0; --count)
            get_size<T>(buf_);
        buf_ = asio::const_buffer(asio::buffer_cast<void const*>(buf), asio::buffer_size(buf) - asio::buffer_size(buf_));
    }

    template<class T>
    lazy_const_iterator<T> lazy_range<T>::begin() const {
        return lazy_const_iterator<T>(0, buf_);
    }

    template<class T>
    lazy_const_iterator<T> lazy_range<T>::end() const {
        return lazy_const_iterator<T>(count_, buf_ + buffer_size());
    }

    using magic_t = std::integral_constant<uint16_t, 0xf00d>;
    using version_t = std::integral_constant<uint16_t, 0xbeef>;

    enum class msg_type_t : uint32_t {
        secdef = 1,
        quote = 2,
    };

    using header_props_t = boost::container::flat_map<boost::string_ref, boost::string_ref>;

    enum class exercise_t : char {
        american = 'A', // default if not present
        european = 'E'
    };

    inline char operator+(exercise_t v) {
        return static_cast<std::underlying_type<exercise_t>::type>(v);
    }

    enum class putcall_t : char {
        put = 'P',
        call = 'C',
    };

    inline char operator+(putcall_t v) {
        return static_cast<std::underlying_type<putcall_t>::type>(v);
    }

    struct decimal_t {
        int8_t exponent_;
        uint32_t mantissa_;

        decimal_t(int8_t e = 0, uint32_t m = 0)
            : exponent_(e)
            , mantissa_(m)
        { }

        operator double() const { return mantissa_ * pow(10, exponent_); }

        friend
        std::ostream& operator<<(std::ostream & stm, decimal_t const& that) {
            return stm << "decimal_t(" << +that.exponent_ << ", " << that.mantissa_ << ")";
        }
    };

    using opt_exercise = optional_field<exercise_t, 0>;
    using opt_quote_ticks = optional_field<decimal_t, 1>;
}

BOOST_FUSION_DEFINE_STRUCT(
    (example), fixed_header,
    (example::magic_t, magic)
    (example::version_t, version)
    (uint32_t, length)
    (example::msg_type_t, msg_type)
)

BOOST_FUSION_DEFINE_STRUCT(
    (example), var_header,
    (example::header_props_t, props)
)

BOOST_FUSION_ADAPT_STRUCT(
    example::decimal_t,
    (int8_t, exponent_)
    (uint32_t, mantissa_)
)

BOOST_FUSION_DEFINE_STRUCT(
    (example), option_t,
    (boost::string_ref, contract_id)
    (boost::string_ref, underlying_id)
    (boost::posix_time::ptime, expiration)
    (example::putcall_t, putcall)
    (example::opt_fields, opts)
    (example::opt_exercise, exercise)
    (example::opt_quote_ticks, quote_ticks)
)

BOOST_FUSION_DEFINE_STRUCT(
    (example), spread_t,
    (boost::string_ref, contract_id)
    (example::lazy<std::vector<example::option_t>>, legs)
)

int main(int argc, char **argv) {
    std::array<char, 1024> buf;
    {
        auto b = asio::buffer(buf) + sizeof(example::fixed_header);
        example::var_header vin;
        vin.props.emplace(boost::string_ref("foo"), boost::string_ref("quux"));
        vin.props.emplace(boost::string_ref("bar"), boost::string_ref("mumble"));
        auto buf_rest = write(b, vin);

        example::option_t oin;
        oin.contract_id = boost::string_ref("GOOG1412I567.5");
        oin.underlying_id = boost::string_ref("GOOG");
        oin.expiration = boost::posix_time::time_from_string("2014-09-13 15:15:00");
        oin.putcall = example::putcall_t::put;
        oin.exercise = example::exercise_t::american;
        oin.quote_ticks = example::decimal_t(-2, 1);

        example::spread_t sin;
        sin.contract_id = boost::string_ref("GOOGFOO");
        sin.legs.get().push_back(oin);
        oin.contract_id = boost::string_ref("GOOG1410V582.5-E");
        oin.putcall = example::putcall_t::call;
        sin.legs.get().push_back(oin);
        buf_rest = write(buf_rest, sin);

        example::fixed_header in;
        in.length = buf.size() - asio::buffer_size(buf_rest);
        in.msg_type = example::msg_type_t::secdef;
        write(asio::buffer(buf), in);
    }

    {
        example::fixed_header out;
        asio::const_buffer buf_rest;
        std::tie(out, buf_rest) = read<example::fixed_header>(asio::buffer(buf));

        example::var_header vout;
        std::tie(vout, buf_rest) = read<example::var_header>(buf_rest);

        example::spread_t sout;
        std::tie(sout, buf_rest) = read<example::spread_t>(buf_rest);
        std::cout << "spread_id = " << sout.contract_id << " - [\n";
        for(auto&& leg: sout.legs.get()) {
            std::cout << "\tcontract_id = " << leg.contract_id << ", "
                      << "underlying_id = " << leg.underlying_id << ", "
                      << "expiration = " << leg.expiration << ", "
                      << "putcall = " << +leg.putcall;
            if (leg.exercise)
                std::cout << ", exercise = " << +*leg.exercise;
            if (leg.quote_ticks)
                std::cout << ", quote_ticks = " << *leg.quote_ticks;
            std::cout << "\n";
        }
        std::cout << "]" << std::endl;
    }
    return 0;
}

