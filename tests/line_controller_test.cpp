#include <cassert>
#include <iostream>
#include "../line_following/line_controller.h"
int main() {
  // Mask bit 0 is physical leftmost; bit 4 is rightmost.
  LineController c;
  assert(c.update(4, 0).left == 0);
  c.start(); auto o = c.update(4, 10);
  assert(o.left > 0 && o.left == o.right);
  o = c.update(2, 20); assert(o.left < o.right);
  c.start(); o = c.update(8, 30); assert(o.left > o.right);
  // Center continuation wins over disconnected decoy marks.
  c.start(); o = c.update(5, 40); assert(o.left == o.right);
  o = c.update(31, 50); assert(o.left == o.right);
  // Two left sensors -> search left, preserving side when all readings disappear.
  c.start(); o = c.update(3, 100); assert(o.left == 0 && o.right > 0);
  o = c.update(0, 200); assert(o.left == 0 && o.right > 0);
  c.update(0, 2099); assert(c.state == LineController::SEARCHING);
  o = c.update(0, 2100); assert(c.state == LineController::STOPPED && o.right == 0);
  // Right corner and stable center reacquisition.
  c.start(); o = c.update(24, 3000); assert(o.left > 0 && o.right == 0);
  c.update(4,3010); c.update(4,3020); o=c.update(4,3030);
  assert(c.state==LineController::FOLLOWING && o.left==o.right);
  // Stop must latch, and a fresh start must discard old search history.
  c.stop(); assert(c.update(4,3040).left==0);
  c.start(); assert(c.update(0,3050).left==0 && c.state==LineController::STOPPED);
  // Off-center detection can select search even without an outer sensor hit.
  c.start(); c.update(8,4000); o=c.update(0,4010);
  assert(o.left>0 && o.right==0);
  // Timer wraparound must not defeat the two-second timeout.
  c.start(); c.update(1,0xffffff00u); o=c.update(0,0x000006d0u);
  assert(c.state==LineController::STOPPED && o.left==0 && o.right==0);
  std::cout << "PASS: steering, decoys, corners, search timeout, reacquisition, stop, timer wrap\n";
}
