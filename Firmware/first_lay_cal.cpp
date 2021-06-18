//! @file
//! @date Jun 10, 2019
//! @author Marek Bel
//! @brief First layer (Z offset) calibration

#include "first_lay_cal.h"
#include "Configuration_prusa.h"
#include "language.h"
#include "Marlin.h"
#include "cmdqueue.h"
#include "mmu.h"
#include <avr/pgmspace.h>

//! @brief Wait for preheat
void lay1cal_wait_preheat()
{
    static const char cmd_preheat_0[] PROGMEM = "M107";
    static const char cmd_preheat_1[] PROGMEM = "M190";
    static const char cmd_preheat_2[] PROGMEM = "M109";
    static const char cmd_preheat_4[] PROGMEM = "G28";
    static const char cmd_preheat_5[] PROGMEM = "G92 E0.0";

    const char * const preheat_cmd[] =
    {
        cmd_preheat_0,
        cmd_preheat_1,
        cmd_preheat_2,
        _T(MSG_M117_V2_CALIBRATION),
        cmd_preheat_4,
        cmd_preheat_5,
    };

    for (uint8_t i = 0; i < (sizeof(preheat_cmd)/sizeof(preheat_cmd[0])); ++i)
    {
        enquecommand_P(preheat_cmd[i]);
    }

}

//! @brief Load filament
//! @param cmd_buffer character buffer needed to format gcodes
//! @param filament filament to use (applies for MMU only)
void lay1cal_load_filament(char *cmd_buffer, uint8_t filament)
{
    if (mmu_enabled)
    {
        enquecommand_P(PSTR("M83"));
        enquecommand_P(PSTR("G1 Y-3.0 F1000.0"));
        enquecommand_P(PSTR("G1 Z0.4 F1000.0"));
        sprintf_P(cmd_buffer, PSTR("T%d"), filament);
        enquecommand(cmd_buffer);
    }

}

void lay1cal_set_flow()
{
    uint8_t nDiameter;
    nDiameter=eeprom_read_byte((uint8_t*)EEPROM_NOZZLE_DIAMETER);

    //char cmd[12];
    //char buffnr[6];
    //dtostrf((double)(nDiameter_um/0.4),3,2,buffnr);
    //sprintf_P(cmd, PSTR("M221 S%s"), buffnr);
    //enquecommand(cmd);

    switch (nDiameter)
    {
    case 25:
        enquecommand_P(PSTR("M221 S62"));
        break;

    case 40:
        enquecommand_P(PSTR("M221 S100"));
        break;
    
    case 60:
        enquecommand_P(PSTR("M221 S150"));
        break;

    case 80:
        enquecommand_P(PSTR("M221 S200"));
        break; 

    default:
        enquecommand_P(PSTR("M221 S100"));
        break;
    }
    
}

//! @brief Print intro line
void lay1cal_intro_line()
{
    static const char cmd_intro_mmu_3[] PROGMEM = "G1 X55.0 E32.0 F1073.0";
    static const char cmd_intro_mmu_4[] PROGMEM = "G1 X5.0 E32.0 F1800.0";
    static const char cmd_intro_mmu_5[] PROGMEM = "G1 X55.0 E8.0 F2000.0";
    static const char cmd_intro_mmu_6[] PROGMEM = "G1 Z0.3 F1000.0";
    static const char cmd_intro_mmu_7[] PROGMEM = "G92 E0.0";
    static const char cmd_intro_mmu_8[] PROGMEM = "G1 X240.0 E25.0  F2200.0";
    static const char cmd_intro_mmu_9[] PROGMEM = "G1 Y-2.0 F1000.0";
    static const char cmd_intro_mmu_10[] PROGMEM = "G1 X55.0 E25 F1400.0";
    static const char cmd_intro_mmu_11[] PROGMEM = "G1 Z0.20 F1000.0";
    static const char cmd_intro_mmu_12[] PROGMEM = "G1 X5.0 E4.0 F1000.0";

    static const char * const intro_mmu_cmd[] PROGMEM =
    {
        cmd_intro_mmu_3,
        cmd_intro_mmu_4,
        cmd_intro_mmu_5,
        cmd_intro_mmu_6,
        cmd_intro_mmu_7,
        cmd_intro_mmu_8,
        cmd_intro_mmu_9,
        cmd_intro_mmu_10,
        cmd_intro_mmu_11,
        cmd_intro_mmu_12,
    };

    if (mmu_enabled)
    {
        for (uint8_t i = 0; i < (sizeof(intro_mmu_cmd)/sizeof(intro_mmu_cmd[0])); ++i)
        {
            enquecommand_P(static_cast<char*>(pgm_read_ptr(&intro_mmu_cmd[i])));
        }
    }
    else
    {
        enquecommand_P(PSTR("G1 X60.0 E9.0 F1000.0"));
        enquecommand_P(PSTR("G1 X100.0 E12.5 F1000.0"));
    }
}

//! @brief Setup for printing meander
void lay1cal_before_meander()
{
    static const char cmd_pre_meander_0[] PROGMEM = "G92 E0.0";
    static const char cmd_pre_meander_1[] PROGMEM = "G21"; //set units to millimeters TODO unsupported command
    static const char cmd_pre_meander_2[] PROGMEM = "G90"; //use absolute coordinates
    static const char cmd_pre_meander_3[] PROGMEM = "M83"; //use relative distances for extrusion TODO: duplicate
    static const char cmd_pre_meander_4[] PROGMEM = "G1 E-1.50000 F2100.00000";
    static const char cmd_pre_meander_5[] PROGMEM = "G1 Z5 F7200.000";
    static const char cmd_pre_meander_6[] PROGMEM = "M204 S1000"; //set acceleration
    static const char cmd_pre_meander_7[] PROGMEM = "G1 F4000";

    static const char * const cmd_pre_meander[] PROGMEM =
    {
            cmd_pre_meander_0,
            cmd_pre_meander_1,
            cmd_pre_meander_2,
            cmd_pre_meander_3,
            cmd_pre_meander_4,
            cmd_pre_meander_5,
            cmd_pre_meander_6,
            cmd_pre_meander_7,
    };

    for (uint8_t i = 0; i < (sizeof(cmd_pre_meander)/sizeof(cmd_pre_meander[0])); ++i)
    {
        enquecommand_P(static_cast<char*>(pgm_read_ptr(&cmd_pre_meander[i])));
    }
}


//! @brief Count extrude length
//!
//! @param layer_height layer height in mm
//! @param extrusion_width extrusion width in mm
//! @param extrusion_length extrusion length in mm
//! @return filament length in mm which needs to be extruded to form line
static constexpr float count_e(float layer_height, float extrusion_width, float extrusion_length)
{
    return (extrusion_length * layer_height * extrusion_width / (M_PI * pow(1.75, 2) / 4));
}

static const float width = 0.4; //!< line width
static const float length = 20 - width; //!< line length
static const float height = 0.2; //!< layer height TODO This is wrong, as current Z height is 0.15 mm
static const float extr = count_e(height, width, length); //!< E axis movement needed to print line
static float xPos1ly[2] = {static_cast<float>(X_MAX_POS)-20.f,20.f};
static const float lengthCSy = (((static_cast<float>(Y_MAX_POS)-20.f)-55.f)/5.f) - width; //!< line length
static float yPos1ly[6] = {static_cast<float>(Y_MAX_POS)-20.f,(static_cast<float>(Y_MAX_POS)-20.f)-lengthCSy,
(static_cast<float>(Y_MAX_POS)-20.f)-(2*lengthCSy),(static_cast<float>(Y_MAX_POS)-20.f)-(3*lengthCSy),
(static_cast<float>(Y_MAX_POS)-20.f)-(4*lengthCSy),(static_cast<float>(Y_MAX_POS)-20.f)-(5*lengthCSy)};
static const float lengthCSx = ((xPos1ly[0]-xPos1ly[1])-width);
static const float lengthCSx_short = lengthCSx - 50.f;
static const float extrX = count_e(height, width,lengthCSx);
static const float extrY = count_e(height, width, lengthCSy);
static const float extrX_short = count_e(height, width, lengthCSx_short);
//static const float extrX_short2 = count_e(height, width, lengthCSx_short2);

//! @brief Print meander
//! @param cmd_buffer character buffer needed to format gcodes
void lay1cal_meander(char *cmd_buffer)
{
    uint8_t y=1;
    uint8_t x=0;
     //print the first part of the meander
    sprintf_P(cmd_buffer, PSTR("G1 X%-2.f Y%-.2f"), xPos1ly[1], yPos1ly[0]);
    enquecommand(cmd_buffer);
    sprintf_P(cmd_buffer, PSTR("G1 Z0.150 F7200.000"));
    enquecommand(cmd_buffer);
    sprintf_P(cmd_buffer, PSTR("G1 F1080"));
    enquecommand(cmd_buffer);
    sprintf_P(cmd_buffer, PSTR( "G1 X70 Y%-2.f E5"),yPos1ly[0]);
    enquecommand(cmd_buffer);
    sprintf_P(cmd_buffer, PSTR( "G1 X%-2.f Y%-.2f E%-.3f"), xPos1ly[0], yPos1ly[0], extrX_short);
    enquecommand(cmd_buffer);

    for(y=1;y<6;y++)
    {
        sprintf_P(cmd_buffer, PSTR( "G1 X%-2.f Y%-.2f E%-.3f"), xPos1ly[x], yPos1ly[y] , extrY);
        enquecommand(cmd_buffer);
        if (x==0) x++; else x--;
        sprintf_P(cmd_buffer, PSTR( "G1 X%-2.f Y%-.2f E%-.3f"), xPos1ly[x], yPos1ly[y] , extrX);
        enquecommand(cmd_buffer);
       
    }
    sprintf_P(cmd_buffer, PSTR("G1 X20 Y35 E%-.3f"), extr);
    enquecommand(cmd_buffer);
}

//! @brief Print square
//!
//! This function needs to be called 16 times for i from 0 to 15.
//!
//! @param cmd_buffer character buffer needed to format gcodes
//! @param i iteration
void lay1cal_square(char *cmd_buffer, uint8_t i)
{
    const float extr_short_segment = count_e(height, width, width);

    static const char fmt1[] PROGMEM = "G1 X%d Y%-.2f E%-.3f";
    static const char fmt2[] PROGMEM = "G1 Y%-.2f E%-.3f";
    sprintf_P(cmd_buffer, fmt1, 40, (35 - i*width * 2), extr);
    enquecommand(cmd_buffer);
    sprintf_P(cmd_buffer, fmt2, (35 - (2 * i + 1)*width), extr_short_segment);
    enquecommand(cmd_buffer);
    sprintf_P(cmd_buffer, fmt1, 20, (35 - (2 * i + 1)*width), extr);
    enquecommand(cmd_buffer);
    sprintf_P(cmd_buffer, fmt2, (35 - (i + 1)*width * 2), extr_short_segment);
    enquecommand(cmd_buffer);
}
