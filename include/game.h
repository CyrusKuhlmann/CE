#pragma once

enum Alliance_ { RED = 1, BLUE = 2 } typedef Alliance;

#define OPPONENTS (ALLIANCE == RED ? BLUE : RED)