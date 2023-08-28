/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

//
// Language Selection Menu
//

#include "../../inc/MarlinConfig.h"

#if HAS_FILLING_MACHINE_MENU

#include "menu_item.h"

#define my_MESH_INSET           25              // Set Mesh bounds as an inset region of the bed  // edited by developium (1)
#define my_GRID_MAX_POINTS_X    8
#define my_GRID_MAX_POINTS_Y    8

#define my_GRID_MAX_CELLS_X (my_GRID_MAX_POINTS_X - 1)
#define my_GRID_MAX_CELLS_Y (my_GRID_MAX_POINTS_Y - 1)

#define my_GRID_MAX_POINTS ((my_GRID_MAX_POINTS_X) * (my_GRID_MAX_POINTS_Y))
// #define my_GRID_LOOP(A,B) LOOP_L_N(A, my_GRID_MAX_POINTS_X) LOOP_L_N(B, my_GRID_MAX_POINTS_Y)

#define my_MESH_MIN_X (_MAX(X_MIN_BED + my_MESH_INSET, X_MIN_POS))  // UBL is careful not to probe off the bed.  It does not
#define my_MESH_MIN_Y (_MAX(Y_MIN_BED + my_MESH_INSET, Y_MIN_POS))  // need NOZZLE_TO_PROBE_OFFSET in the mesh dimensions
#define my_MESH_MAX_X (_MIN(X_MAX_BED - (my_MESH_INSET), X_MAX_POS))
#define my_MESH_MAX_Y (_MIN(Y_MAX_BED - (my_MESH_INSET), Y_MAX_POS))

#define my_MESH_X_DIST (float(my_MESH_MAX_X - (my_MESH_MIN_X)) / (my_GRID_MAX_CELLS_X))
#define my_MESH_Y_DIST (float(my_MESH_MAX_Y - (my_MESH_MIN_Y)) / (my_GRID_MAX_CELLS_Y))

#define my_GRIDPOS(A,N) (my_MESH_MIN_##A + N * (my_MESH_##A##_DIST))

// static uint8_t my_n_edit_pts = 1;
static int8_t my_x_plot = 0, my_y_plot = 0; // May be negative during move

static const float
my_mesh_index_to_xpos[my_GRID_MAX_POINTS_X] PROGMEM = ARRAY_N(my_GRID_MAX_POINTS_X,
  my_GRIDPOS(X,  0), my_GRIDPOS(X,  1), my_GRIDPOS(X,  2), my_GRIDPOS(X,  3),
  my_GRIDPOS(X,  4), my_GRIDPOS(X,  5), my_GRIDPOS(X,  6), my_GRIDPOS(X,  7),
  my_GRIDPOS(X,  8), my_GRIDPOS(X,  9), my_GRIDPOS(X, 10), my_GRIDPOS(X, 11),
  my_GRIDPOS(X, 12), my_GRIDPOS(X, 13), my_GRIDPOS(X, 14), my_GRIDPOS(X, 15)
),
my_mesh_index_to_ypos[my_GRID_MAX_POINTS_Y] PROGMEM = ARRAY_N(my_GRID_MAX_POINTS_Y,
  my_GRIDPOS(Y,  0), my_GRIDPOS(Y,  1), my_GRIDPOS(Y,  2), my_GRIDPOS(Y,  3),
  my_GRIDPOS(Y,  4), my_GRIDPOS(Y,  5), my_GRIDPOS(Y,  6), my_GRIDPOS(Y,  7),
  my_GRIDPOS(Y,  8), my_GRIDPOS(Y,  9), my_GRIDPOS(Y, 10), my_GRIDPOS(Y, 11),
  my_GRIDPOS(Y, 12), my_GRIDPOS(Y, 13), my_GRIDPOS(Y, 14), my_GRIDPOS(Y, 15)
);


static float my_get_mesh_x(const uint8_t i) {
  return i < (my_GRID_MAX_POINTS_X) ? pgm_read_float(&my_mesh_index_to_xpos[i]) : my_MESH_MIN_X + i * (my_MESH_X_DIST);
}
static float my_get_mesh_y(const uint8_t i) {
  return i < (my_GRID_MAX_POINTS_Y) ? pgm_read_float(&my_mesh_index_to_ypos[i]) : my_MESH_MIN_Y + i * (my_MESH_Y_DIST);
}

/**
 * UBL LCD Map Movement
 */
void my_ubl_map_move_to_xy() {
  const xy_pos_t xy = { my_get_mesh_x(my_x_plot), my_get_mesh_y(my_y_plot) };

  // Some printers have unreachable areas in the mesh. Skip the move if unreachable.
  if (!position_is_reachable(xy)) return;

  // Use the built-in manual move handler to move to the mesh point.
  ui.manual_move.set_destination(xy);
  ui.manual_move.soon(ALL_AXES_ENUM);
}

/**
 * UBL LCD "radar" map
 */
void my_ubl_map_screen() {
  // static millis_t next_move = 0;
  // const millis_t ms = millis();

  uint8_t x, y;

  if (ui.first_page) {

    if (ui.use_click()) {
      // _lcd_ubl_map_edit_cmd();// On click send "G29 P4 ..." to edit the Z value
      // ui.return_to_status();
      // ui.go_back();
      // ui.goto_previous_screen();
      // ui.synchronize(GET_TEXT_F(MSG_PROBE_WIZARD_MOVING));
      // ui.wait_for_move = true;
      ui.goto_previous_screen_no_defer();
      return;
    }

    ui.defer_status_screen();

    #if IS_KINEMATIC
      // Index of the mesh point upon entry
      const int32_t old_pos_index = grid_index(my_x_plot, my_y_plot);
      // Direction from new (unconstrained) encoder value
      const int8_t step_dir = int32_t(ui.encoderPosition) < old_pos_index ? -1 : 1;
    #endif

    do {
      // Now, keep the encoder position within range
      if (int32_t(ui.encoderPosition) < 0) ui.encoderPosition = my_GRID_MAX_POINTS + TERN(TOUCH_SCREEN, ui.encoderPosition, -1);
      if (int32_t(ui.encoderPosition) > my_GRID_MAX_POINTS - 1) ui.encoderPosition = TERN(TOUCH_SCREEN, ui.encoderPosition - my_GRID_MAX_POINTS, 0);

      // Draw the grid point based on the encoder
      x = ui.encoderPosition % (my_GRID_MAX_POINTS_X);
      y = ui.encoderPosition / (my_GRID_MAX_POINTS_X);

      // Validate if needed
      #if IS_KINEMATIC
        const xy_pos_t xy = { my_get_mesh_x(x), my_get_mesh_y(y) };
        if (position_is_reachable(xy)) break; // Found a valid point
        ui.encoderPosition += step_dir;       // Test the next point
      #endif
    } while (ENABLED(IS_KINEMATIC));

    // // Determine number of points to edit
    // #if IS_KINEMATIC
    //   my_n_edit_pts = 9; // TODO: Delta accessible edit points
    // #else
    //   const bool xc = WITHIN(x, 1, (my_GRID_MAX_POINTS_X) - 2),
    //              yc = WITHIN(y, 1, (my_GRID_MAX_POINTS_Y) - 2);
    //   my_n_edit_pts = yc ? (xc ? 9 : 6) : (xc ? 6 : 4); // Corners
    // #endif

    // Refresh is also set by encoder movement
    //if (int32_t(ui.encoderPosition) != grid_index(x, y))
    //  ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
  }

  // Draw the grid point based on the encoder
  x = ui.encoderPosition % (my_GRID_MAX_POINTS_X);
  y = ui.encoderPosition / (my_GRID_MAX_POINTS_X);

  if (ui.should_draw()) ui.ubl_plot(x, y);

  // Add a move if needed to match the grid point
  if (x != my_x_plot || y != my_y_plot) {
    my_x_plot = x; my_y_plot = y;   // The move is always posted, so update the grid point now
    my_ubl_map_move_to_xy();     // Sets up a "manual move"
    ui.refresh(LCDVIEW_CALL_REDRAW_NEXT); // Clean up a half drawn box
  }
}

void my_radar(){
  ui.goto_screen(my_ubl_map_screen);
}

// M300 S440 P50
void say_beep(uint8_t rep=1, uint16_t tonnee=100, uint16_t timeee=100) {
  for (size_t i = 0; i < rep; i++)
  {
    // queue.enqueue_now(F("M300 S4000 P1000"));
    char buffer[50];
    snprintf_P(buffer, sizeof(buffer), PSTR("M300 S%d P%d"), tonnee*10, timeee*10);
    queue.enqueue_one_now(buffer);
  }
}
namespace Language_en {
      LSTR MSG_GO_BACK                        = _UxGT("Go Back");
      LSTR MSG_BEEP                           = _UxGT("Beep");
      LSTR MSG_tonnee                           = _UxGT("Tone");
      LSTR MSG_timeee                           = _UxGT("Time");
      LSTR MSG_Count                           = _UxGT("Count");
      LSTR MSG_RADAR                           = _UxGT("Radar");
}
void menu_filling_machine() {

  ui.defer_status_screen();
  START_MENU();
  BACK_ITEM(MSG_GO_BACK);

  static uint16_t myFreq=100;
  static uint16_t myTime=100;
  static uint8_t myCount=5;
  // EDIT_ITEM(uint8, MSG_INTENSITY_R, &leds.color.r, 0, 255, leds.update, true);
  EDIT_ITEM_FAST(uint16_3, MSG_tonnee, &myFreq, 0, 100);
  EDIT_ITEM_FAST(uint16_4, MSG_timeee, &myTime, 0, 100);
  EDIT_ITEM_FAST(uint8, MSG_Count, &myCount, 0, 100);
  GCODES_ITEM(MSG_AUTO_HOME, F("G28"));
  ACTION_ITEM(MSG_BEEP, []{ say_beep(myCount,myFreq,myTime); } );
  ACTION_ITEM(MSG_RADAR, my_radar);
  // ACTION_ITEM(MSG_UBL_MESH_EDIT, _ubl_goto_map_screen);
  
  END_MENU();
}

#endif // HAS_FILLING_MACHINE_MENU
