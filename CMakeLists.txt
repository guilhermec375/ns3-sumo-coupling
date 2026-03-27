build_lib(
  LIBNAME sumo-coupling
  SOURCE_FILES
    model/sumo-client.cc
    model/sumo-manager.cc
  HEADER_FILES
    model/sumo-client.h
    model/sumo-manager.h
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
)