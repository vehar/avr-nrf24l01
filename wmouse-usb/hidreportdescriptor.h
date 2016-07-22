char usbHidReportDescriptor[] = { /* USB report descriptor, size must match usbconfig.h */
	
	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	0x09, 0x01,                    //   USAGE (Pointer)
	0xa1, 0x01,                    // COLLECTION (Application)
	0x85, 0x02,                    //   REPORT_ID (2)
	0x09, 0x01,                    //   USAGE (Pointer)
	0xA1, 0x00,                    //   COLLECTION (Physical)
	0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
	0x66,0x00,0x00,                //     UNIT (None)
	0x55,0x00,                     //     UNIT_EXPONENT (00)
	0x09, 0x30,                    //     USAGE (X)
	0x09, 0x31,                    //     USAGE (Y)
	0x16, 0x01, 0x80,              //     LOGICAL_MINIMUM (-32767)
//	0x16, 0x00, 0x00,              //     LOGICAL_MINIMUM (-32767)
	0x26, 0xff, 0x7f,              //     LOGICAL_MAXIMUM (32767)
	0x36, 0x01, 0x80,              //     PHYSICAL_MINIMUM (-32767)
//	0x36, 0x00, 0x00,              //     PHYSICAL_MINIMUM (-32767)
	0x46, 0xff, 0x7f,              //     PHYSICAL_MAXIMUM (32767)
	0x75, 0x10,                    //     REPORT_SIZE (16)
	0x95, 0x02,                    //     REPORT_COUNT (2)
	0x81, 0x02,                    //     INPUT (Data,Var,Rel)
	0xC0,                          //   END_COLLECTION
	0xC0,                          // END COLLECTION
	
	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	0x09, 0x02,                    // USAGE (Mouse)
	0xa1, 0x01,                    // COLLECTION (Application)
	0x85, 0x01,                    //   REPORT_ID (1)
	0x09, 0x01,                    //   USAGE (Pointer)
	0xA1, 0x00,                    //   COLLECTION (Physical)
	0x05, 0x09,                    //     USAGE_PAGE (Button)
	0x19, 0x01,                    //     USAGE_MINIMUM
	0x29, 0x03,                    //     USAGE_MAXIMUM
	0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
	0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
	0x75, 0x01,                    //     REPORT_SIZE (1)
	0x95, 0x03,                    //     REPORT_COUNT (3)
	0x81, 0x02,                    //     INPUT (Data,Var,Abs)
	0x75, 0x05,                    //     REPORT_SIZE (1)
	0x95, 0x01,                    //     REPORT_COUNT (5)
	0x81, 0x03,                    //     INPUT (Const,Var,Abs)
	0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
	0x09, 0x30,                    //     USAGE (X)
	0x09, 0x31,                    //     USAGE (Y)
	0x09, 0x38,                    //     USAGE (Wheel)
	0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
	0x25, 0x7F,                    //     LOGICAL_MAXIMUM (127)
	0x75, 0x08,                    //     REPORT_SIZE (8)
	0x95, 0x03,                    //     REPORT_COUNT (3)
	0x81, 0x06,                    //     INPUT (Data,Var,Rel)
	0xC0,                          //   END_COLLECTION
	0xC0,                          // END COLLECTION
	
};

// char usbHidReportDescriptor[] = { /* USB report descriptor, size must match usbconfig.h */
// 	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
// 	0x09, 0x02,                    // USAGE (Mouse)
// 	0xa1, 0x01,                    // COLLECTION (Application)
// 	0x85, 0x01,                    //   REPORT_ID (1)
// 	0x09, 0x01,                    //   USAGE (Pointer)
// 	0xA1, 0x00,                    //   COLLECTION (Physical)
// 	0x05, 0x09,                    //     USAGE_PAGE (Button)
// 	0x19, 0x01,                    //     USAGE_MINIMUM
// 	0x29, 0x03,                    //     USAGE_MAXIMUM
// 	0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
// 	0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
// 	0x95, 0x03,                    //     REPORT_COUNT (3)
// 	0x75, 0x01,                    //     REPORT_SIZE (1)
// 	0x81, 0x02,                    //     INPUT (Data,Var,Abs)
// 	0x95, 0x01,                    //     REPORT_COUNT (1)
// 	0x75, 0x05,                    //     REPORT_SIZE (5)
// 	0x81, 0x03,                    //     INPUT (Const,Var,Abs)
// 	0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
// 	0x09, 0x30,                    //     USAGE (X)
// 	0x09, 0x31,                    //     USAGE (Y)
// 	0x09, 0x38,                    //     USAGE (Wheel)
// 	0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
// 	0x25, 0x7F,                    //     LOGICAL_MAXIMUM (127)
// 	0x75, 0x08,                    //     REPORT_SIZE (8)
// 	0x95, 0x03,                    //     REPORT_COUNT (3)
// 	0x81, 0x06,                    //     INPUT (Data,Var,Rel)
// 	0xC0,                          //   END_COLLECTION
// 	0xC0,                          // END COLLECTION
// 
// };


/* This is the same report descriptor as seen in a Logitech mouse. The data
 * described by this descriptor consists of 4 bytes:
 *      .  .  .  .  . B2 B1 B0 .... one byte with mouse button states
 *     X7 X6 X5 X4 X3 X2 X1 X0 .... 8 bit signed relative coordinate x
 *     Y7 Y6 Y5 Y4 Y3 Y2 Y1 Y0 .... 8 bit signed relative coordinate y
 *     W7 W6 W5 W4 W3 W2 W1 W0 .... 8 bit signed relative coordinate wheel
 */


typedef struct{
    char reportId;
    uint8_t   buttonMask;
    int8_t    dx;
    int8_t    dy;
    int8_t    dWheel;
}report1_t;

typedef struct{
    char reportId;
    uint16_t    x;
    uint16_t    y;
//    int16_t    dWheel;
}report2_t;


