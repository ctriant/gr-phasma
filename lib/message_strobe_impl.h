/* -*- c++ -*- */
/*
 * Copyright 2012-2013 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_PHASMA_MESSAGE_STROBE_IMPL_H
#define INCLUDED_PHASMA_MESSAGE_STROBE_IMPL_H

#include <phasma/message_strobe.h>
#include <pthread.h>

namespace gr
{
  namespace phasma
  {

  class PHASMA_API message_strobe_impl : public message_strobe
  {
  private:
    boost::shared_ptr<boost::thread> d_thread;
    bool d_finished;
    float d_period_ms;
    float d_min;
    float d_max;
    float d_step;
    pmt::pmt_t d_msg;

    void run();

  public:
    message_strobe_impl(float period_ms, float min, float max,
        float step);
    ~message_strobe_impl();

    void set_period(float period_ms) {d_period_ms = period_ms;}
    float period() const {return d_period_ms;}

    // Overloading these to start and stop the internal thread that
    // periodically produces the message.
    bool start();
    bool stop();
  };

}
/* namespace phasma */
} /* namespace gr */

#endif /* INCLUDED_PHASMA_MESSAGE_STROBE_IMPL_H */
