#ifndef _Visual_H_
#define _Visual_H_

#include <iostream>
#include "Data.h"
using namespace std;
class Data;

class View{
	public:
		int count;
		void refresh(Data *data);
};
#endif