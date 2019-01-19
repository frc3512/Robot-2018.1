// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <cstddef>

/**
 * Makes a bounded queue with a fixed memory size
 */
template <class T, size_t N>
class circular_buffer {
public:
    typedef T value_type;
    typedef value_type& reference;
    typedef const value_type& const_reference;
    typedef value_type* pointer;
    typedef size_t size_type;
    typedef std::forward_iterator_tag iterator_category;
    typedef std::ptrdiff_t difference_type;

    size_type size() const;
    T& front();
    const T& front() const;
    T& back();
    const T& back() const;
    void push_front(T value);
    void push_back(T value);
    T pop_front();
    T pop_back();
    void reset();

    T& operator[](size_t index);
    const T& operator[](size_t index) const;

private:
    std::array<T, N> m_data;

    // Index of element at front of buffer
    size_t m_front = 0;

    // Number of elements used in buffer
    size_t m_length = 0;

    size_t ModuloInc(size_t index);
    size_t ModuloDec(size_t index);
};

#include "circular_buffer.inc"
