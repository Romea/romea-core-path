#ifndef ROMEA_CORE_PATH_CUMULATIVESUM_HPP_
#define ROMEA_CORE_PATH_CUMULATIVESUM_HPP_

// std
#include <vector>
#include <memory>
#include <cstdlib>

namespace romea {

template <typename T, typename Allocator>
class CumulativeSum
{
public:
  using Vector = std::vector<T, Allocator> ;
  using iterator = typename Vector::iterator;
  using const_iterator = typename Vector::const_iterator;

public :

  CumulativeSum();

  explicit CumulativeSum(const T & initialValue);

  const Vector & data()const;

  void increment(const double & delta);

  const T & initialValue()const;

  const T & finalValue()const;

  const T & operator[](size_t n) const;

  void reserve(size_t n);

  size_t size()const;

  void clear();

  const_iterator begin() const;

  const_iterator end() const;

private :

  Vector cumsum_;
};


//-----------------------------------------------------------------------------
template <typename T, typename Allocator>
CumulativeSum<T, Allocator>::CumulativeSum():
  CumulativeSum(0)
{
}

//-----------------------------------------------------------------------------
template <typename T, typename Allocator>
CumulativeSum<T, Allocator>::CumulativeSum(const T & initialValue)
  : cumsum_(1, initialValue)
{
}

//-----------------------------------------------------------------------------
template <typename T, typename Allocator>
void CumulativeSum<T, Allocator>::increment(const double & delta)
{
  cumsum_.push_back(cumsum_.back()+delta);
}

//-----------------------------------------------------------------------------
template <typename T, typename Allocator>
const T & CumulativeSum<T, Allocator>::initialValue()const
{
  return cumsum_.front();
}

//-----------------------------------------------------------------------------
template <typename T, typename Allocator>
const T & CumulativeSum<T, Allocator>::finalValue()const
{
  return cumsum_.back();
}

//-----------------------------------------------------------------------------
template <typename T, typename Allocator>
const T & CumulativeSum<T, Allocator>::operator[](size_t n) const
{
  return cumsum_[n];
}

//-----------------------------------------------------------------------------
template <typename T, typename Allocator>
void CumulativeSum<T, Allocator>::reserve(size_t n)
{
  cumsum_.reserve(n);
}

//-----------------------------------------------------------------------------
template <typename T, typename Allocator>
size_t CumulativeSum<T, Allocator>::size() const
{
  return cumsum_.size();
}

//-----------------------------------------------------------------------------
template <typename T, typename Allocator>
void CumulativeSum<T, Allocator>::clear()
{
  cumsum_.clear();
  cumsum_.push_back(0);
}

//-----------------------------------------------------------------------------
template <typename T, typename Allocator>
const typename CumulativeSum<T, Allocator>::Vector &
CumulativeSum<T, Allocator>::data()const
{
  return cumsum_;
}

//-----------------------------------------------------------------------------
template <typename T, typename Allocator>
typename CumulativeSum<T, Allocator>::const_iterator
CumulativeSum<T, Allocator>::begin() const
{
  return cumsum_.begin();
}

//-----------------------------------------------------------------------------
template <typename T, typename Allocator>
typename CumulativeSum<T, Allocator>::const_iterator
CumulativeSum<T, Allocator>::end() const
{
  return cumsum_.end();
}

}  // namespace romea


#endif  // ROMEA_CORE_PATH_CUMULATIVESUM_HPP_
