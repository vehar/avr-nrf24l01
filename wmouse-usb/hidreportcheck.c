#include <stdio.h>
#include <stdint.h>

#include "hidreportdescriptor.h"

int main()
{
	printf("#define USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH    %d\n",sizeof(usbHidReportDescriptor));
	return 0;
};
