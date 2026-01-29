/*
  Copyright (c) 2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifdef __cplusplus

#ifndef _RING_BUFFER_
#define _RING_BUFFER_

#include <stdint.h>
#include <type_traits>

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.

#ifndef SERIAL_BUFFER_SIZE
#define SERIAL_BUFFER_SIZE 350
#endif

template <int N, typename T = uint8_t>
class RingBufferN
{
  public:
    T _aucBuffer[N] ;
    volatile int _iHead ;
    volatile int _iTail ;

  public:
    RingBufferN( void ) ;
    bool store( const T& c ) ;
    bool read( T& out ) ;
    bool peek( T& out ) const ;
    void store_char( uint8_t c ) ;
    void clear();
    int read_char();
    int available();
    int availableForStore();
    int peek();
    bool isFull();

  private:
    int nextIndex(int index);
};

typedef RingBufferN<SERIAL_BUFFER_SIZE> RingBuffer;


template <int N, typename T>
RingBufferN<N>::RingBufferN( void )
{
    for (int i = 0; i < N; ++i)
      _aucBuffer[i] = T{};
    clear();
}

template <int N, typename T>
bool RingBufferN<N, T>::store( const T& c )
{
  int i = nextIndex(_iHead);

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if ( i != _iTail )
  {
    _aucBuffer[_iHead] = c ;
    _iHead = i ;
    return true;
  }
  return false;
}

template <int N, typename T>
bool RingBufferN<N, T>::read( T& out )
{
  if(_iTail == _iHead)
    return false;

  out = _aucBuffer[_iTail];
  _iTail = nextIndex(_iTail);
  return true;
}

template <int N, typename T>
bool RingBufferN<N, T>::peek( T& out ) const
{
  if(_iTail == _iHead)
    return false;

  out = _aucBuffer[_iTail];
  return true;
}

template <int N, typename T>
void RingBufferN<N, T>::store_char( uint8_t c )
{
  static_assert(std::is_same<T, uint8_t>::value, "store_char only valid for uint8_t buffers");
  (void)store(static_cast<T>(c));
}

template <int N, typename T>
void RingBufferN<N, T>::clear()
{
  _iHead = 0;
  _iTail = 0;
}

template <int N, typename T>
int RingBufferN<N, T>::read_char()
{
  static_assert(std::is_same<T, uint8_t>::value, "read_char only valid for uint8_t buffers");
  uint8_t value;
  if (!read(value))
    return -1;
  return value;
}

template <int N, typename T>
int RingBufferN<N, T>::available()
{
  int delta = _iHead - _iTail;

  if(delta < 0)
    return N + delta;
  else
    return delta;
}

template <int N, typename T>
int RingBufferN<N, T>::availableForStore()
{
  if (_iHead >= _iTail)
    return N - 1 - _iHead + _iTail;
  else
    return _iTail - _iHead - 1;
}

template <int N, typename T>
int RingBufferN<N, T>::peek()
{
  static_assert(std::is_same<T, uint8_t>::value, "peek() only valid for uint8_t buffers");
  uint8_t value;
  if (!peek(value))
    return -1;

  return value;
}

template <int N, typename T>
int RingBufferN<N, T>::nextIndex(int index)
{
  return (uint32_t)(index + 1) % N;
}

template <int N, typename T>
bool RingBufferN<N, T>::isFull()
{
  return (nextIndex(_iHead) == _iTail);
}

#endif /* _RING_BUFFER_ */

#endif /* __cplusplus */
