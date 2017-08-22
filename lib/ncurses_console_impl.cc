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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "ncurses_console_impl.h"
#include <pmt/pmt.h>

namespace gr
{
  namespace phasma
  {

    ncurses_console::sptr
    ncurses_console::make (const std::vector<std::string> &labels)
    {
      return gnuradio::get_initial_sptr (new ncurses_console_impl (labels));
    }

    /*
     * The private constructor
     */
    ncurses_console_impl::ncurses_console_impl (
        const std::vector<std::string> &labels) :
            gr::block ("ncurses_console", gr::io_signature::make (0, 0, 0),
                       gr::io_signature::make (0, 0, 0)),
            d_running (true)
    {
      message_port_register_in (pmt::mp ("in"));

      if (labels.size () <= 0) {
        throw std::runtime_error (
            "ncurses_console: Prediction labels incorrectly set");
      }

      d_labels = labels;
      d_nlabels = labels.size ();

//      for (size_t i = 0; i < d_nlabels; i++) {
//        d_confusion_matrix.push_back (
//            std::vector<std::pair<int, float>> (d_nlabels));
//      }

      d_print_thread = boost::shared_ptr<boost::thread> (
          new boost::thread (
              boost::bind (&ncurses_console_impl::msg_handler, this)));

      console_init ();

    }

    /*
     * Our virtual destructor.
     */
    ncurses_console_impl::~ncurses_console_impl ()
    {
    }

    bool
    ncurses_console_impl::stop ()
    {
      std::cout << "Closing" << std::endl;
      d_running = false;
      d_print_thread->join ();
      curs_set (1);
      clear ();
      for (size_t i = 0; i < d_nlabels + 1; i++) {
        for (size_t j = 0; j < d_nlabels + 1; j++) {
          delwin (d_confusion_matrix_win[i][j]);
        }
      }
      delwin (d_logo_win);
      endwin ();
      return true;
    }

    void
    ncurses_console_impl::update_confusion_matrix_screen (
        phasma_confusion_matrix_type_t* matrix)
    {
      for (size_t i = 1; i <= d_nlabels; i++) {
        for (size_t j = 1; j <= d_nlabels; j++) {
          float v = (*matrix)[i - 1][j - 1].second;
          if (v > 0) {
            mvwprintw (d_confusion_matrix_win[i][j], 0, 0, "%0.2f", v);
            wrefresh (d_confusion_matrix_win[i][j]);
          }
        }
      }
    }

    void
    ncurses_console_impl::msg_handler ()
    {
      pmt::pmt_t msg;
      pmt::pmt_t val;
      float noise_floor;
      phasma_confusion_matrix_type_t* cnfn_matrix;

      while (d_running) {
        try {
          // Access the message queue
          msg = delete_head_nowait (pmt::mp ("in"));
          if (!eq (msg, pmt::pmt_t ())) {
            val = dict_ref (msg, pmt::intern ("NOISE_FLOOR_UPD"), pmt::PMT_NIL);
            if (!eq (val, pmt::PMT_NIL)) {
              noise_floor = to_float (val);
              update_noise_floor_info (noise_floor);
            }
            val = dict_ref (msg, pmt::intern ("CNFN_MATRIX_UPD"), pmt::PMT_NIL);
            if (!eq (val, pmt::PMT_NIL)) {
              cnfn_matrix = (phasma_confusion_matrix_type_t *) pmt::blob_data (
                  val);
              update_confusion_matrix_screen (cnfn_matrix);
            }
            usleep (30000);
          }
        }
        catch (pmt::out_of_range&) {
          /* You are out of range so break */
        }
      }
    }

    void
    ncurses_console_impl::update_noise_floor_info (float noise_floor)
    {
      mvwprintw (d_system_info_win, 2, 2, "%s %.1f %s", "Mean noise-floor:",
                 noise_floor, "dB");
      wclrtoeol (d_system_info_win);
      wrefresh (d_system_info_win);
    }

    void
    ncurses_console_impl::console_init ()
    {
      size_t i = 0;
      size_t j;
      int pos;
      struct tm *tm;
      time_t t;
      char str_time[300];

      t = time (NULL);
      tm = localtime (&t);
      strftime (str_time, sizeof(str_time), "%d-%m-%Y at %H:%M:%S", tm);

      sleep (1);
      initscr ();
      init_logo ();
      init_system_info ();

      for (size_t i = 0; i < d_nlabels + 1; i++) {
        for (size_t j = 0; j < d_nlabels + 1; j++) {
          if (!i && !j) {
            d_confusion_matrix_win[i][j] = create_newwin (1, COLS / 10,
            LOGO_HEIGHT + SYSTEM_INFO_HEIGHT + 1 + i,
                                                          j);
            box (d_confusion_matrix_win[i][j], ' ', ' ');
            wborder (d_confusion_matrix_win[i][j], ' ', ' ', ' ', ' ', ' ', ' ',
                     ' ', ' ');
            wprintw (d_confusion_matrix_win[i][j], "%s", " ");
          }
          else if (!i && j > 0) {
            d_confusion_matrix_win[i][j] = create_newwin (1, COLS / 10,
            LOGO_HEIGHT + SYSTEM_INFO_HEIGHT + 1 + i,
                                                          (COLS / 10) * j);
            box (d_confusion_matrix_win[i][j], ' ', ' ');
            wborder (d_confusion_matrix_win[i][j], ' ', ' ', ' ', ' ', ' ', ' ',
                     ' ', ' ');
            wprintw (d_confusion_matrix_win[i][j], "%s",
                     d_labels[j - 1].c_str ());
          }
          else if (!j && i > 0) {
            d_confusion_matrix_win[i][j] = create_newwin (1, COLS / 10,
            LOGO_HEIGHT + SYSTEM_INFO_HEIGHT + 1 + i,
                                                          j);
            box (d_confusion_matrix_win[i][j], ' ', ' ');
            wborder (d_confusion_matrix_win[i][j], ' ', ' ', ' ', ' ', ' ', ' ',
                     ' ', ' ');
            wprintw (d_confusion_matrix_win[i][j], "%s",
                     d_labels[i - 1].c_str ());
          }
          else if (j && i) {
            d_confusion_matrix_win[i][j] = create_newwin (1, COLS / 10,
            LOGO_HEIGHT + 1 + SYSTEM_INFO_HEIGHT + i,
                                                          (COLS / 10) * j);
            box (d_confusion_matrix_win[i][j], ' ', ' ');
            wborder (d_confusion_matrix_win[i][j], ' ', ' ', ' ', ' ', ' ', ' ',
                     ' ', ' ');
            wprintw (d_confusion_matrix_win[i][j], "%s", " ");
          }
          wrefresh (d_confusion_matrix_win[i][j]);
        }
      }
    }

  } /* namespace phasma */
} /* namespace gr */

