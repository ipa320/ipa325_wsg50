#pragma once

#include <stdlib.h>
#include "FRIBasicDriverCPP.h"

#ifndef __testerMain__
#define __testerMain__

void testGetterMethods(FRIBasicDriverCPP *fd);
void testGetterCompleteInformation(FRIBasicDriverCPP *fd);

// test movements
void testCmdVelJoint(FRIBasicDriverCPP *fd, double *velPos);
bool testCmdVelCart(FRIBasicDriverCPP *fd);
bool testCartTrajecotry(FRIBasicDriverCPP *fd);
bool findHoleTester(FRIBasicDriverCPP *fd);

void printFloatArray(float *ar, int size);
void printDoubleArray(double *ar, int size);

#endif
