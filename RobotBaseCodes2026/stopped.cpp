#include "Arduino.h"
#include "stopped.h"
#include "tiller.h"

void Stopped::begin() {
  tiller_->println("--------STOPPED---------");

  tiller_->_motors->writeAllMotors(0.0, 0.0, 0.0);
}

void Stopped::end() {
}

void Stopped::poll() {
  const unsigned long t = millis();

  if (t - last_millis_ > 5000) {
tiller_->println("            |\\|\\,'\\,'\\ ,.");
    tiller_->println("            )        ;' |,'");
    tiller_->println("           /              |,'|,.");
    tiller_->println("          /                  ` /__");
    tiller_->println("         ,'                    ,-'");
    tiller_->println("        ,'                    :");
    tiller_->println("       (_                     '");
    tiller_->println("     ,'                      ;");
    tiller_->println("     |---._ ,'     .        '");
    tiller_->println("     :   o Y---.__  ;      ;");
    tiller_->println("     /`,\"\"-|     o`.|     /");
    tiller_->println("    ,  `._  `.    ,'     ;");
    tiller_->println("    ;         `\"\"'      ;");
    tiller_->println("   /                   -'.");
    tiller_->println("   \\                   G  )");
    tiller_->println("    `-.__________,   `._,'");
    tiller_->println("            (`   `     |)\\");
    tiller_->println("           / `.       ,'  \\");
    tiller_->println("          /    `-----'     \\");
    tiller_->println("         /");
    
    // Update the timer so it only prints once every second
    last_millis_ = t; 
  }
}