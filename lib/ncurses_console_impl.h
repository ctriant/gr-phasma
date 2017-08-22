/* -*- c++ -*- */
/*
 * Copyright 2017 Kostis Triantafyllakis - ctriant.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_PHASMA_NCURSES_CONSOLE_IMPL_H
#define INCLUDED_PHASMA_NCURSES_CONSOLE_IMPL_H

#include <phasma/ncurses_console.h>
#include <phasma/utils/ncurses_utils.h>
#include <pthread.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>

namespace gr
{
  namespace phasma
  {

    class ncurses_console_impl : public ncurses_console
    {
    private:
      boost::shared_ptr<boost::thread> d_print_thread;

      bool d_running;
      std::vector<std::string> d_labels;

      size_t d_nlabels;
      phasma_confusion_matrix_type_t* d_confusion_matrix;

    boost::condition_variable
    d_start_print_cond;

  public:
    ncurses_console_impl (const std::vector<std::string> &labels);
    ~ncurses_console_impl ();

    bool
    stop ();

    void
    update_confusion_matrix_screen (phasma_confusion_matrix_type_t* matrix);

    void
    msg_handler ();

    void
    update_noise_floor_info (float noise_floor);

    void
    console_init ();

  };

}
 // namespace phasma
}// namespace gr

#endif /* INCLUDED_PHASMA_NCURSES_CONSOLE_IMPL_H */

