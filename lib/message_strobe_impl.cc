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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <iostream>
#include <random>
#include "message_strobe_impl.h"

namespace gr
{
  namespace phasma
  {

    message_strobe::sptr
    message_strobe::make (float period_ms,
                          float min, float max, float step)
    {
      return gnuradio::get_initial_sptr (
          new message_strobe_impl (period_ms, min, max, step));
    }

    message_strobe_impl::message_strobe_impl (float period_ms,
                                              float min, float max, float step) :
    gr::block ("message_strobe", gr::io_signature::make (0, 0, 0),
               gr::io_signature::make (0, 0, 0)),
            d_finished (false),
            d_period_ms (period_ms),
            d_min(min),
            d_max(max),
            d_step(step)
    {
      message_port_register_out (pmt::mp ("strobe"));

      message_port_register_in (pmt::mp ("set_msg"));
    }

    message_strobe_impl::~message_strobe_impl ()
    {
    }

    bool
    message_strobe_impl::start ()
    {
      // NOTE: d_finished should be something explicitely thread safe. But since
      // nothing breaks on concurrent access, I'll just leave it as bool.
      d_finished = false;

      d_thread = boost::shared_ptr<boost::thread> (
                new boost::thread (
                    boost::bind (&message_strobe_impl::run, this)));


      return block::start ();
    }

    bool
    message_strobe_impl::stop ()
    {
      // Shut down the thread
      d_finished = true;
      d_thread->interrupt ();
      d_thread->join ();

      return block::stop ();
    }

    void
    message_strobe_impl::run ()
    {
      std::vector<float> choices;
      while (!d_finished) {
        boost::this_thread::sleep (
            boost::posix_time::milliseconds (d_period_ms));
        if (d_finished) {
          return;
        }
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(d_min, -100e3);
//        std::uniform_int_distribution<> dis2(100e3, d_max);
//        choices.push_back(dis(gen));
//        choices.push_back(dis2(gen));
//        std::random_shuffle(choices.begin(), choices.end());
        message_port_pub (pmt::mp ("strobe"), pmt::from_float (dis(gen)));
      }
    }

  } /* namespace blocks */
} /* namespace gr */
