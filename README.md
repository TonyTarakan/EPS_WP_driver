# EPS8266 custom driver for Sierra Wireless WP85

Creates two network inetrfaces and routes pakages between them


 .----------.   .----------.
 |          |   |          |
 |   mesh   |   |   wifi   |
 |          |   |          |
 '----------'   '----------'
       ^             ^
       |             |
        \           /
     .----------------.       .---------.
     |                |  SPI  |         |
     |     Driver     |<----->|   ESP   |
     |                |       |         |
     '----------------'       '---------' 