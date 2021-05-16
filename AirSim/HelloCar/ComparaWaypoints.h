#pragma once
using namespace segment;
#include <stdlib.h>
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <fstream>
#include <math.h>
#include <sstream>
#include <cstring>
#include <string>
#include "Waypoints.h"
#include "LateralControl.h"
#include "LongitudinalControl.h"
#include "pnmfile.h"
image<uchar>* carregar_mapa(const char* filenameCerto) {
	image<uchar>* mapa = loadPBM(filenameCerto);
	return mapa;
}
void salvar_mapa(image<uchar>* mapa, int x, int y) {
	mapa->access[150 + x][150 + y] = 0;
}

image<uchar>* novo_mapa(const char* filenameCerto) {
	image<uchar>* mapa = new segment::image<unsigned char>(300, 300, false);
	mapa->init(1);
	return mapa;
}
bool compara_mapa(image<uchar>* mapa, int x, int y) {
	if (mapa->access[150 + x][150 + y] == 0) {
		return false;
	}
	else return true;
}
