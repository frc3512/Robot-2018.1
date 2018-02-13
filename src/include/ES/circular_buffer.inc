// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <algorithm>

/**
 * Returns number of elements in buffer
 */
template <class T, size_t N>
typename circular_buffer<T, N>::size_type circular_buffer<T, N>::size() const {
    return m_length;
}

/**
 * Returns value at front of buffer
 */
template <class T, size_t N>
T& circular_buffer<T, N>::front() {
    return (*this)[0];
}

/**
 * Returns value at front of buffer
 */
template <class T, size_t N>
const T& circular_buffer<T, N>::front() const {
    return (*this)[0];
}

/**
 * Returns value at back of buffer
 */
template <class T, size_t N>
T& circular_buffer<T, N>::back() {
    // If there are no elements in the buffer, do nothing
    if (m_length == 0) {
        return 0;
    }

    return m_data[(m_front + m_length - 1) % N];
}

/**
 * Returns value at back of buffer
 */
template <class T, size_t N>
const T& circular_buffer<T, N>::back() const {
    // If there are no elements in the buffer, do nothing
    if (m_length == 0) {
        return 0;
    }

    return m_data[(m_front + m_length - 1) % N];
}

/**
 * Push new value onto front of the buffer. The value at the back is overwritten
 * if the buffer is full.
 */
template <class T, size_t N>
void circular_buffer<T, N>::push_front(T value) {
    m_front = ModuloDec(m_front);

    m_data[m_front] = value;

    if (m_length < m_data.size()) {
        m_length++;
    }
}

/**
 * Push new value onto back of the buffer. The value at the front is overwritten
 * if the buffer is full.
 */
template <class T, size_t N>
void circular_buffer<T, N>::push_back(T value) {
    m_data[(m_front + m_length) % N] = value;

    if (m_length < N) {
        m_length++;
    } else {
        // Increment front if buffer is full to maintain size
        m_front = ModuloInc(m_front);
    }
}

/**
 * Pop value at front of buffer.
 */
template <class T, size_t N>
T circular_buffer<T, N>::pop_front() {
    // If there are no elements in the buffer, do nothing
    if (m_length == 0) {
        return T();
    }

    T& temp = m_data[m_front];
    m_front = ModuloInc(m_front);
    m_length--;
    return temp;
}

/**
 * Pop value at back of buffer.
 */
template <class T, size_t N>
T circular_buffer<T, N>::pop_back() {
    // If there are no elements in the buffer, do nothing
    if (m_length == 0) {
        return 0;
    }

    m_length--;
    return m_data[(m_front + m_length) % N];
}

/**
 * Sets internal buffer contents to zero.
 */
template <class T, size_t N>
void circular_buffer<T, N>::reset() {
    std::fill(m_data.begin(), m_data.end(), 0);
    m_front = 0;
    m_length = 0;
}

/**
 * @return Element at index starting from front of buffer.
 */
template <class T, size_t N>
T& circular_buffer<T, N>::operator[](size_t index) {
    return m_data[(m_front + index) % N];
}

/**
 * @return Element at index starting from front of buffer.
 */
template <class T, size_t N>
const T& circular_buffer<T, N>::operator[](size_t index) const {
    return m_data[(m_front + index) % N];
}

/**
 * Increment an index modulo the length of the buffer.
 *
 * @return The result of the modulo operation.
 */
template <class T, size_t N>
size_t circular_buffer<T, N>::ModuloInc(size_t index) {
    return (index + 1) % N;
}

/**
 * Decrement an index modulo the length of the buffer.
 *
 * @return The result of the modulo operation.
 */
template <class T, size_t N>
size_t circular_buffer<T, N>::ModuloDec(size_t index) {
    if (index == 0) {
        return N - 1;
    } else {
        return index - 1;
    }
}
