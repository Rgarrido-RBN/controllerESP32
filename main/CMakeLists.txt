set(include_dirs "inc"
                 "."
   )

set(src_dir "./"
            "src/"
            "src/gpio/"
            "src/button/"
            "src/bluetooth/"
   )

idf_component_register(
                       SRC_DIRS     "${src_dir}"

                       INCLUDE_DIRS "${include_dirs}"

                       REQUIRES     esp_hid
                                    driver
                      )
