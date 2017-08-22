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

#ifndef INCLUDED_PHASMA_NCURSES_CONSOLE_H
#define INCLUDED_PHASMA_NCURSES_CONSOLE_H

#include <phasma/api.h>
#include <gnuradio/block.h>

namespace gr
{
  namespace phasma
  {

    /*!
     * \brief <+description of block+>
     * \ingroup phasma
     *
     */
    class PHASMA_API ncurses_console : virtual public gr::block
    {
    public:
      typedef boost::shared_ptr<ncurses_console> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of phasma::ncurses_console.
       *
       * To avoid accidental use of raw pointers, phasma::ncurses_console's
       * constructor is in a private implementation
       * class. phasma::ncurses_console::make is the public interface for
       * creating new instances.
       */
      static sptr
      make (const std::vector<std::string> &labels);
    };

  } // namespace phasma
} // namespace gr

#endif /* INCLUDED_PHASMA_NCURSES_CONSOLE_H */

