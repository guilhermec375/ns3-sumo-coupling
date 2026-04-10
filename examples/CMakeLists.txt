build_lib_example(
  NAME simple-vanet
  SOURCE_FILES simple-vanet.cc
  LIBRARIES_TO_LINK
    ${libsumo-coupling}
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libinternet}
    ${libapplications}
    ${libwifi}
)

build_lib_example(
  NAME minimal-vanet
  SOURCE_FILES minimal-vanet.cc
  LIBRARIES_TO_LINK
    ${libsumo-coupling}
    ${libcore}
    ${libnetwork}
    ${libmobility}
)