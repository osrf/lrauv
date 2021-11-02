cmake_minimum_required(VERSION 3.5)

project(lrauv_description)

install(DIRECTORY
  hooks
  models
  DESTINATION share)
