#ifndef BT_CIRCULAR_BUFFER_H_
#define BT_CIRCULAR_BUFFER_H_

#include <stdexcept>
#include <vector>

template<typename T>
class bt_circular_buffer
{
public:

  bt_circular_buffer()
    : begin_pos_(0), current_pos_(0)
  {
    try
    {
      data_ = new T[BUFFER_SIZE];
    }
    catch (std::bad_alloc)
    {
      delete[] data_;
    }
  }

  ~bt_circular_buffer()
  {
    delete[] data_;
  }

  void push(T const& data)
  {
    data_[current_pos_] = data;

    current_pos_ = increase_pos(current_pos_);
    if (++size_ >= BUFFER_SIZE)
    {
      size_ = BUFFER_SIZE;
    }

    if (current_pos_ <= begin_pos_)
    {
      begin_pos_ = increase_pos(begin_pos_);
    }
  }

  std::vector<T> read_all()
  {
    std::vector<T> return_data;
    int read_begin = current_pos_;

    if (current_pos_ > begin_pos_ || (current_pos_ == 0 && begin_pos_ == BUFFER_SIZE - 1))
    {
      for (int i = 0; i < current_pos_; ++i)
      {
        return_data.push_back(data_[i]);
      }
    }
    else
    {
      for (int i = 0; i < BUFFER_SIZE; ++i, ++read_begin)
      {
        return_data.push_back(data_[read_begin % BUFFER_SIZE]);
      }
    }

    return return_data;
  }

  T& pop(void)
  {
    if (current_pos_ - 1 < 0)
    {
      return data_[BUFFER_SIZE - 1];
    }
    else
    {
      return data_[current_pos_ - 1];
    }
  }

  int size_max()
  {
    return BUFFER_SIZE;
  }

  int size_used()
  {
    if (current_pos_ > begin_pos_ || (current_pos_ == 0 && begin_pos_ == BUFFER_SIZE - 1))
    {
      return current_pos_;
    }
    else
    {
      return size_max();
    }
  }


private:

  static const int BUFFER_SIZE = 132000;

  T* data_;
  int current_pos_;

  size_t size_;
  int begin_pos_;

  int increase_pos(int pos)
  {
    return pos = (pos + 1) % BUFFER_SIZE;
  }
};

#endif
