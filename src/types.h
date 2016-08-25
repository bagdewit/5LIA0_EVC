#ifndef TYPES_H
#define TYPES_H

#define MAX_PACKETS_IN_AVR 25

typedef enum {
    INIT,
    PROCESSINGIMG,
    CREATEQUEUE,
	FLUSH,
	SENDING,
	ACKERROR,
	BROKEN
} global_state;

typedef struct {
    float x, y;
} coord_t;

typedef struct {
    float x, y;
} direct_t;

typedef struct {
    float rho, phi;
} rotat_t;

typedef struct {
    bool in_use;
    char id;
    char instr_type[4];
    char ldir, rdir;
    int lspd :16;
    int rspd :16;
    int duration :16;
} packet;

enum Lanetype { l, r, lf, rf, f, lrf, lr, dead, in_corner, none2};

typedef struct {
    Lanetype type; 
    int tl_x, tl_y;
    int w, h;
    int size;
	int approachAngle;
} Lane;

enum Signtype { lturn, fturn, rturn, stopsign, uturn, none };



typedef struct {
    Signtype type; 
    int tl_x, tl_y;
    int w, h;
    int size;
} Sign;

typedef struct {
    int serial_con;
    coord_t car_location;
    direct_t car_direction;
    rotat_t panel_rotation;
    rotat_t panel_at_zero; //calibration at car_direction == 0,0
    int clock_last_picture;
    int nclocks_new_picture; //should be smaller at corners and the like

    packet all_packets[MAX_PACKETS_IN_AVR];
    char packetstring[25];

    Sign red_sign;
    Sign blue_sign;
    Sign yellow_sign;

	Lanetype lastTurnFlag;
	Lanetype lastTurnFlag_other;
	
    Lane cur_lane;
	
	bool redSignFlag;
	bool redSignFlagEx;
	int yellowSignFlag;
	Signtype blueSignFlag;
	Lanetype lastDecisionFlag;
	
	int noAck_count;
	int noAck2_count;
	
	int file_i2c;
	int inst_cnt;
	int addr;
	
	int whereToGo;
	int whereToGoCount;
} state_data;

typedef global_state (*state_function) (state_data *data);

#endif
