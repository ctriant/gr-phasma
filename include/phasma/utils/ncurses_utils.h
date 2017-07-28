/*
 * Copyright 2011 Free Software Foundation, Inc.
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

#ifndef INCLUDED_PHASMA_UTILS_NCURSES_UTILS_H_
#define INCLUDED_PHASMA_UTILS_NCURSES_UTILS_H_

#include <ncurses.h>

#define CONFUSION_MATRIX_SIZE 10
#define LOGO_HEIGHT 10
#define SYSTEM_INFO_HEIGHT 5

typedef struct _win_border_struct
{
  chtype ls, rs, ts, bs, tl, tr, bl, br;
} WIN_BORDER;

typedef struct _WIN_struct
{

  int startx, starty;
  int height, width;
  WIN_BORDER border;
} WIN;

static inline void
init_win_params (WIN *p_win)
{
  p_win->height = 3;
  p_win->width = 10;
  p_win->starty = (LINES - p_win->height) / 2;
  p_win->startx = (COLS - p_win->width) / 2;

  p_win->border.ls = '|';
  p_win->border.rs = '|';
  p_win->border.ts = '-';
  p_win->border.bs = '-';
  p_win->border.tl = '+';
  p_win->border.tr = '+';
  p_win->border.bl = '+';
  p_win->border.br = '+';

}

static inline WINDOW*
create_newwin (int height, int width, int starty, int startx)
{
  WINDOW *local_win;

  local_win = newwin (height, width, starty, startx);
  box (local_win, 0, 0); /* 0, 0 gives default characters
   * for the vertical and horizontal
   * lines			*/
  wborder (local_win, '|', '|', '-', '-', '+', '+', '+', '+');
  wrefresh (local_win); /* Show that box 		*/

  return local_win;
}

static inline void
destroy_win (WINDOW *local_win)
{
  /* box(local_win, ' ', ' '); : This won't produce the desired
   * result of erasing the window. It will leave it's four corners
   * and so an ugly remnant of window.
   */
  wborder (local_win, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ');
  /* The parameters taken are
   * 1. win: the window on which to operate
   * 2. ls: character to be used for the left side of the window
   * 3. rs: character to be used for the right side of the window
   * 4. ts: character to be used for the top side of the window
   * 5. bs: character to be used for the bottom side of the window
   * 6. tl: character to be used for the top left corner of the window
   * 7. tr: character to be used for the top right corner of the window
   * 8. bl: character to be used for the bottom left corner of the window
   * 9. br: character to be used for the bottom right corner of the window
   */
  wrefresh (local_win);
  delwin (local_win);
}

static inline void
create_box (WIN *p_win, bool flag)
{
  int i, j;
  int x, y, w, h;

  x = p_win->startx;
  y = p_win->starty;
  w = p_win->width;
  h = p_win->height;

  if (flag == TRUE) {
    mvaddch(y, x, p_win->border.tl);
    mvaddch(y, x + w, p_win->border.tr);
    mvaddch(y + h, x, p_win->border.bl);
    mvaddch(y + h, x + w, p_win->border.br);
    mvhline(y, x + 1, p_win->border.ts, w - 1);
    mvhline(y + h, x + 1, p_win->border.bs, w - 1);
    mvvline(y + 1, x, p_win->border.ls, h - 1);
    mvvline(y + 1, x + w, p_win->border.rs, h - 1);
  }
  else
    for (j = y; j <= y + h; ++j)
      for (i = x; i <= x + w; ++i)
	mvaddch(j, i, ' ');

  refresh ();
}

static const char *d_ascii_logo =
".______    __    __       ___           _______..___  ___.      ___     \n"
"|   _  \\  |  |  |  |     /   \\         /       ||   \\/   |     /   \\    \n"
"|  |_)  | |  |__|  |    /  ^  \\       |   (----`|  \\  /  |    /  ^  \\   \n"
"|   ___/  |   __   |   /  /_\\  \\       \\   \\    |  |\\/|  |   /  /_\\  \\  \n"
"|  |      |  |  |  |  /  _____  \\  .----)   |   |  |  |  |  /  _____  \\ \n"
"| _|      |__|  |__| /__/     \\__\\ |_______/    |__|  |__| /__/     \\__\\ \n";

static WINDOW *d_logo_win;
static WINDOW *d_system_info_win;
static WINDOW *d_confusion_matrix_win[CONFUSION_MATRIX_SIZE][CONFUSION_MATRIX_SIZE];

static inline void
init_logo ()
{
  d_logo_win = create_newwin (LOGO_HEIGHT, COLS, 0, 0);
  box (d_logo_win, ' ', ' ');
  wborder (d_logo_win, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ');
  /* Print the logo and a geeky message at a constant window*/
  wprintw (d_logo_win, "%s", d_ascii_logo);
  wprintw (d_logo_win, "\nA GNURadio Out-of-Tree Module for Automatic Modulation Classification");
  wrefresh (d_logo_win);

}

static inline void
init_system_info ()
{
  d_system_info_win = create_newwin (SYSTEM_INFO_HEIGHT, COLS, LOGO_HEIGHT, 0);
  box (d_logo_win, '-', '-');
  wborder (d_logo_win, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ');
  /* Print the logo and a geeky message at a constant window*/
  wprintw (d_system_info_win, "%s", "System information");
  mvwprintw(d_system_info_win, 2, 2, "%s", "Noise floor:  Estimation algorithm is running...");
  mvwprintw(d_system_info_win, 3, 2, "%s", "Classification method:  Random Forest");
  wrefresh (d_system_info_win);

}

static inline void
update_noise_floor (float noise_floor)
{
  mvwprintw(d_system_info_win, 2, 2, "%s %f", "Noise floor: ", noise_floor);
  wrefresh (d_system_info_win);
}

#endif /* INCLUDED_PHASMA_UTILS_NCURSES_UTILS_H_ */
