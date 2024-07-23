// Benchmark "adder" written by ABC on Wed Jul 17 13:59:13 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n345,
    new_n348, new_n350, new_n352, new_n354;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nand02aa1d28x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  and002aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o(new_n101));
  inv040aa1d32x5               g006(.a(\a[3] ), .o1(new_n102));
  inv040aa1d32x5               g007(.a(\b[2] ), .o1(new_n103));
  nand22aa1n12x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nand22aa1n09x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand22aa1n03x5               g010(.a(new_n104), .b(new_n105), .o1(new_n106));
  nand42aa1n08x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nor042aa1d18x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nand02aa1d28x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  oaih12aa1n12x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  oa0022aa1n06x5               g015(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n111));
  tech160nm_fioai012aa1n03p5x5 g016(.a(new_n111), .b(new_n110), .c(new_n106), .o1(new_n112));
  norp02aa1n12x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nand22aa1n09x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nor022aa1n16x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nand02aa1n06x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nona23aa1n09x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .out0(new_n118));
  nor022aa1n08x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nand42aa1n03x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  nanb02aa1n03x5               g025(.a(new_n119), .b(new_n120), .out0(new_n121));
  nor042aa1n02x5               g026(.a(new_n118), .b(new_n121), .o1(new_n122));
  nona23aa1n12x5               g027(.a(new_n112), .b(new_n122), .c(new_n117), .d(new_n101), .out0(new_n123));
  nano23aa1n03x7               g028(.a(new_n113), .b(new_n115), .c(new_n116), .d(new_n114), .out0(new_n124));
  orn002aa1n24x5               g029(.a(\a[5] ), .b(\b[4] ), .o(new_n125));
  oaoi03aa1n09x5               g030(.a(\a[6] ), .b(\b[5] ), .c(new_n125), .o1(new_n126));
  inv000aa1d42x5               g031(.a(\a[7] ), .o1(new_n127));
  inv000aa1d42x5               g032(.a(\b[6] ), .o1(new_n128));
  aoai13aa1n02x7               g033(.a(new_n114), .b(new_n113), .c(new_n127), .d(new_n128), .o1(new_n129));
  aobi12aa1n06x5               g034(.a(new_n129), .b(new_n124), .c(new_n126), .out0(new_n130));
  oai112aa1n04x5               g035(.a(new_n123), .b(new_n130), .c(\b[8] ), .d(\a[9] ), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n99), .b(new_n131), .c(new_n100), .out0(\s[10] ));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nand42aa1d28x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  aoai13aa1n06x5               g040(.a(new_n98), .b(new_n97), .c(new_n131), .d(new_n100), .o1(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n134), .c(new_n135), .out0(\s[11] ));
  norb02aa1n02x5               g042(.a(new_n135), .b(new_n133), .out0(new_n138));
  aoi012aa1n02x5               g043(.a(new_n97), .b(new_n131), .c(new_n100), .o1(new_n139));
  nano22aa1n02x4               g044(.a(new_n139), .b(new_n98), .c(new_n138), .out0(new_n140));
  norp02aa1n24x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1d28x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanb02aa1n02x5               g047(.a(new_n141), .b(new_n142), .out0(new_n143));
  nano22aa1n02x4               g048(.a(new_n140), .b(new_n134), .c(new_n143), .out0(new_n144));
  inv000aa1d42x5               g049(.a(new_n138), .o1(new_n145));
  oaoi13aa1n06x5               g050(.a(new_n143), .b(new_n134), .c(new_n136), .d(new_n145), .o1(new_n146));
  norp02aa1n03x5               g051(.a(new_n146), .b(new_n144), .o1(\s[12] ));
  nona23aa1d18x5               g052(.a(new_n142), .b(new_n135), .c(new_n133), .d(new_n141), .out0(new_n148));
  nor002aa1n16x5               g053(.a(\b[8] ), .b(\a[9] ), .o1(new_n149));
  oaih12aa1n06x5               g054(.a(new_n98), .b(new_n149), .c(new_n97), .o1(new_n150));
  tech160nm_fioai012aa1n03p5x5 g055(.a(new_n142), .b(new_n141), .c(new_n133), .o1(new_n151));
  oai012aa1d24x5               g056(.a(new_n151), .b(new_n148), .c(new_n150), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nano23aa1n09x5               g058(.a(new_n133), .b(new_n141), .c(new_n142), .d(new_n135), .out0(new_n154));
  nano23aa1d15x5               g059(.a(new_n149), .b(new_n97), .c(new_n98), .d(new_n100), .out0(new_n155));
  nand22aa1n12x5               g060(.a(new_n155), .b(new_n154), .o1(new_n156));
  aoai13aa1n06x5               g061(.a(new_n153), .b(new_n156), .c(new_n123), .d(new_n130), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nand02aa1n06x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n159), .b(new_n157), .c(new_n160), .o1(new_n161));
  xnrb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oaoi13aa1n12x5               g067(.a(new_n101), .b(new_n111), .c(new_n110), .d(new_n106), .o1(new_n163));
  nor043aa1n02x5               g068(.a(new_n117), .b(new_n118), .c(new_n121), .o1(new_n164));
  oaib12aa1n06x5               g069(.a(new_n129), .b(new_n117), .c(new_n126), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n156), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n165), .c(new_n163), .d(new_n164), .o1(new_n167));
  nor002aa1d32x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand02aa1n12x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nona23aa1n09x5               g074(.a(new_n169), .b(new_n160), .c(new_n159), .d(new_n168), .out0(new_n170));
  oaih12aa1n12x5               g075(.a(new_n169), .b(new_n168), .c(new_n159), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n171), .b(new_n170), .c(new_n167), .d(new_n153), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nand22aa1n12x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nanb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  nor022aa1n16x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand02aa1d08x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanb02aa1n02x5               g084(.a(new_n178), .b(new_n179), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n180), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(new_n181), .b(new_n174), .c(new_n172), .d(new_n177), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n174), .o1(new_n183));
  nano23aa1n03x7               g088(.a(new_n159), .b(new_n168), .c(new_n169), .d(new_n160), .out0(new_n184));
  inv000aa1d42x5               g089(.a(new_n171), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n177), .b(new_n185), .c(new_n157), .d(new_n184), .o1(new_n186));
  tech160nm_fiaoi012aa1n02p5x5 g091(.a(new_n180), .b(new_n186), .c(new_n183), .o1(new_n187));
  norp02aa1n02x5               g092(.a(new_n187), .b(new_n182), .o1(\s[16] ));
  nano23aa1n06x5               g093(.a(new_n174), .b(new_n178), .c(new_n179), .d(new_n175), .out0(new_n189));
  nona23aa1n09x5               g094(.a(new_n189), .b(new_n155), .c(new_n148), .d(new_n170), .out0(new_n190));
  nona23aa1n09x5               g095(.a(new_n179), .b(new_n175), .c(new_n174), .d(new_n178), .out0(new_n191));
  nor002aa1n03x5               g096(.a(new_n191), .b(new_n170), .o1(new_n192));
  oai012aa1n02x5               g097(.a(new_n179), .b(new_n178), .c(new_n174), .o1(new_n193));
  tech160nm_fioai012aa1n05x5   g098(.a(new_n193), .b(new_n191), .c(new_n171), .o1(new_n194));
  aoi012aa1d24x5               g099(.a(new_n194), .b(new_n152), .c(new_n192), .o1(new_n195));
  aoai13aa1n12x5               g100(.a(new_n195), .b(new_n190), .c(new_n123), .d(new_n130), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nano22aa1d15x5               g102(.a(new_n156), .b(new_n184), .c(new_n189), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n165), .c(new_n163), .d(new_n164), .o1(new_n199));
  inv040aa1d32x5               g104(.a(\a[17] ), .o1(new_n200));
  inv040aa1d28x5               g105(.a(\b[16] ), .o1(new_n201));
  nanp02aa1n04x5               g106(.a(new_n201), .b(new_n200), .o1(new_n202));
  and002aa1n02x5               g107(.a(\b[16] ), .b(\a[17] ), .o(new_n203));
  aoi013aa1n02x4               g108(.a(new_n203), .b(new_n199), .c(new_n195), .d(new_n202), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1n04x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nanp02aa1n04x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  nano23aa1n06x5               g112(.a(new_n206), .b(new_n203), .c(new_n202), .d(new_n207), .out0(new_n208));
  inv000aa1n06x5               g113(.a(new_n208), .o1(new_n209));
  aoai13aa1n12x5               g114(.a(new_n207), .b(new_n206), .c(new_n200), .d(new_n201), .o1(new_n210));
  aoai13aa1n02x7               g115(.a(new_n210), .b(new_n209), .c(new_n199), .d(new_n195), .o1(new_n211));
  xorb03aa1n02x5               g116(.a(new_n211), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nand42aa1n16x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  nor002aa1d32x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nand02aa1n16x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  aoi112aa1n02x5               g124(.a(new_n214), .b(new_n219), .c(new_n211), .d(new_n216), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n214), .o1(new_n221));
  oaoi03aa1n02x5               g126(.a(\a[18] ), .b(\b[17] ), .c(new_n202), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n216), .b(new_n222), .c(new_n196), .d(new_n208), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n219), .o1(new_n224));
  aoi012aa1n03x5               g129(.a(new_n224), .b(new_n223), .c(new_n221), .o1(new_n225));
  nor002aa1n02x5               g130(.a(new_n225), .b(new_n220), .o1(\s[20] ));
  nona23aa1d18x5               g131(.a(new_n218), .b(new_n215), .c(new_n214), .d(new_n217), .out0(new_n227));
  norb02aa1d27x5               g132(.a(new_n208), .b(new_n227), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  oai012aa1d24x5               g134(.a(new_n218), .b(new_n217), .c(new_n214), .o1(new_n230));
  oai012aa1d24x5               g135(.a(new_n230), .b(new_n227), .c(new_n210), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n02x7               g137(.a(new_n232), .b(new_n229), .c(new_n199), .d(new_n195), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  nand02aa1n06x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(new_n237));
  nor042aa1n06x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nanp02aa1n04x5               g143(.a(\b[21] ), .b(\a[22] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  aoi112aa1n03x4               g145(.a(new_n235), .b(new_n240), .c(new_n233), .d(new_n237), .o1(new_n241));
  inv000aa1n09x5               g146(.a(new_n235), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n237), .b(new_n231), .c(new_n196), .d(new_n228), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n240), .o1(new_n244));
  aoi012aa1n03x5               g149(.a(new_n244), .b(new_n243), .c(new_n242), .o1(new_n245));
  norp02aa1n03x5               g150(.a(new_n245), .b(new_n241), .o1(\s[22] ));
  nano23aa1d15x5               g151(.a(new_n235), .b(new_n238), .c(new_n239), .d(new_n236), .out0(new_n247));
  oaoi03aa1n12x5               g152(.a(\a[22] ), .b(\b[21] ), .c(new_n242), .o1(new_n248));
  aoi012aa1d18x5               g153(.a(new_n248), .b(new_n231), .c(new_n247), .o1(new_n249));
  nano23aa1n03x7               g154(.a(new_n214), .b(new_n217), .c(new_n218), .d(new_n215), .out0(new_n250));
  nano22aa1n02x4               g155(.a(new_n209), .b(new_n250), .c(new_n247), .out0(new_n251));
  inv000aa1n02x5               g156(.a(new_n251), .o1(new_n252));
  aoai13aa1n02x7               g157(.a(new_n249), .b(new_n252), .c(new_n199), .d(new_n195), .o1(new_n253));
  nor002aa1d24x5               g158(.a(\b[22] ), .b(\a[23] ), .o1(new_n254));
  nand02aa1d04x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n255), .b(new_n254), .out0(new_n256));
  inv000aa1d42x5               g161(.a(new_n249), .o1(new_n257));
  aoi112aa1n02x5               g162(.a(new_n257), .b(new_n256), .c(new_n196), .d(new_n251), .o1(new_n258));
  aoi012aa1n02x5               g163(.a(new_n258), .b(new_n253), .c(new_n256), .o1(\s[23] ));
  tech160nm_finor002aa1n05x5   g164(.a(\b[23] ), .b(\a[24] ), .o1(new_n260));
  nand02aa1d04x5               g165(.a(\b[23] ), .b(\a[24] ), .o1(new_n261));
  norb02aa1n02x5               g166(.a(new_n261), .b(new_n260), .out0(new_n262));
  aoi112aa1n03x4               g167(.a(new_n254), .b(new_n262), .c(new_n253), .d(new_n256), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n254), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n256), .b(new_n257), .c(new_n196), .d(new_n251), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n262), .o1(new_n266));
  aoi012aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n264), .o1(new_n267));
  nor002aa1n02x5               g172(.a(new_n267), .b(new_n263), .o1(\s[24] ));
  nano23aa1n06x5               g173(.a(new_n254), .b(new_n260), .c(new_n261), .d(new_n255), .out0(new_n269));
  nano32aa1n02x4               g174(.a(new_n209), .b(new_n269), .c(new_n250), .d(new_n247), .out0(new_n270));
  inv000aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  nanp02aa1n02x5               g176(.a(new_n250), .b(new_n222), .o1(new_n272));
  nand02aa1n02x5               g177(.a(new_n269), .b(new_n247), .o1(new_n273));
  oai022aa1n02x5               g178(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n274));
  aoi022aa1n06x5               g179(.a(new_n269), .b(new_n248), .c(new_n261), .d(new_n274), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n273), .c(new_n272), .d(new_n230), .o1(new_n276));
  inv000aa1n02x5               g181(.a(new_n276), .o1(new_n277));
  aoai13aa1n02x7               g182(.a(new_n277), .b(new_n271), .c(new_n199), .d(new_n195), .o1(new_n278));
  xorb03aa1n02x5               g183(.a(new_n278), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g184(.a(\b[24] ), .b(\a[25] ), .o1(new_n280));
  tech160nm_fixorc02aa1n04x5   g185(.a(\a[25] ), .b(\b[24] ), .out0(new_n281));
  xorc02aa1n12x5               g186(.a(\a[26] ), .b(\b[25] ), .out0(new_n282));
  aoi112aa1n03x4               g187(.a(new_n280), .b(new_n282), .c(new_n278), .d(new_n281), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n280), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n281), .b(new_n276), .c(new_n196), .d(new_n270), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n282), .o1(new_n286));
  aoi012aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n284), .o1(new_n287));
  nor002aa1n02x5               g192(.a(new_n287), .b(new_n283), .o1(\s[26] ));
  nona23aa1n02x4               g193(.a(new_n239), .b(new_n236), .c(new_n235), .d(new_n238), .out0(new_n289));
  nona23aa1n02x4               g194(.a(new_n261), .b(new_n255), .c(new_n254), .d(new_n260), .out0(new_n290));
  nand22aa1n04x5               g195(.a(new_n282), .b(new_n281), .o1(new_n291));
  nona32aa1n09x5               g196(.a(new_n228), .b(new_n291), .c(new_n290), .d(new_n289), .out0(new_n292));
  nanp02aa1n02x5               g197(.a(\b[25] ), .b(\a[26] ), .o1(new_n293));
  inv000aa1n02x5               g198(.a(new_n291), .o1(new_n294));
  oai022aa1n02x5               g199(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n295));
  aoi022aa1n02x7               g200(.a(new_n276), .b(new_n294), .c(new_n293), .d(new_n295), .o1(new_n296));
  aoai13aa1n04x5               g201(.a(new_n296), .b(new_n292), .c(new_n199), .d(new_n195), .o1(new_n297));
  xorb03aa1n02x5               g202(.a(new_n297), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nanp02aa1n02x5               g203(.a(\b[26] ), .b(\a[27] ), .o1(new_n299));
  inv000aa1d42x5               g204(.a(\a[28] ), .o1(new_n300));
  inv000aa1d42x5               g205(.a(\b[27] ), .o1(new_n301));
  nanp02aa1n02x5               g206(.a(new_n301), .b(new_n300), .o1(new_n302));
  nanp02aa1n02x5               g207(.a(\b[27] ), .b(\a[28] ), .o1(new_n303));
  inv040aa1n06x5               g208(.a(new_n292), .o1(new_n304));
  nor002aa1n02x5               g209(.a(new_n290), .b(new_n289), .o1(new_n305));
  nand02aa1d04x5               g210(.a(new_n231), .b(new_n305), .o1(new_n306));
  nanp02aa1n02x5               g211(.a(new_n295), .b(new_n293), .o1(new_n307));
  aoai13aa1n12x5               g212(.a(new_n307), .b(new_n291), .c(new_n306), .d(new_n275), .o1(new_n308));
  nor042aa1d18x5               g213(.a(\b[26] ), .b(\a[27] ), .o1(new_n309));
  aoi112aa1n02x7               g214(.a(new_n309), .b(new_n308), .c(new_n196), .d(new_n304), .o1(new_n310));
  nano32aa1n03x5               g215(.a(new_n310), .b(new_n303), .c(new_n302), .d(new_n299), .out0(new_n311));
  nand42aa1n02x5               g216(.a(new_n196), .b(new_n304), .o1(new_n312));
  nona22aa1n03x5               g217(.a(new_n312), .b(new_n308), .c(new_n309), .out0(new_n313));
  aoi022aa1n02x7               g218(.a(new_n313), .b(new_n299), .c(new_n302), .d(new_n303), .o1(new_n314));
  norp02aa1n03x5               g219(.a(new_n314), .b(new_n311), .o1(\s[28] ));
  nano32aa1n02x4               g220(.a(new_n309), .b(new_n303), .c(new_n299), .d(new_n302), .out0(new_n316));
  aoai13aa1n06x5               g221(.a(new_n316), .b(new_n308), .c(new_n196), .d(new_n304), .o1(new_n317));
  oaoi03aa1n12x5               g222(.a(new_n300), .b(new_n301), .c(new_n309), .o1(new_n318));
  xorc02aa1n12x5               g223(.a(\a[29] ), .b(\b[28] ), .out0(new_n319));
  inv000aa1d42x5               g224(.a(new_n319), .o1(new_n320));
  tech160nm_fiaoi012aa1n05x5   g225(.a(new_n320), .b(new_n317), .c(new_n318), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n318), .o1(new_n322));
  aoi112aa1n02x5               g227(.a(new_n319), .b(new_n322), .c(new_n297), .d(new_n316), .o1(new_n323));
  nor002aa1n02x5               g228(.a(new_n321), .b(new_n323), .o1(\s[29] ));
  xorb03aa1n02x5               g229(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb02aa1n02x5               g230(.a(new_n316), .b(new_n320), .out0(new_n326));
  aoai13aa1n06x5               g231(.a(new_n326), .b(new_n308), .c(new_n196), .d(new_n304), .o1(new_n327));
  oao003aa1n12x5               g232(.a(\a[29] ), .b(\b[28] ), .c(new_n318), .carry(new_n328));
  tech160nm_fixorc02aa1n03p5x5 g233(.a(\a[30] ), .b(\b[29] ), .out0(new_n329));
  inv000aa1d42x5               g234(.a(new_n329), .o1(new_n330));
  aoi012aa1n03x5               g235(.a(new_n330), .b(new_n327), .c(new_n328), .o1(new_n331));
  inv000aa1d42x5               g236(.a(new_n328), .o1(new_n332));
  aoi112aa1n03x4               g237(.a(new_n329), .b(new_n332), .c(new_n297), .d(new_n326), .o1(new_n333));
  norp02aa1n03x5               g238(.a(new_n331), .b(new_n333), .o1(\s[30] ));
  and003aa1n02x5               g239(.a(new_n316), .b(new_n329), .c(new_n319), .o(new_n335));
  oaoi03aa1n06x5               g240(.a(\a[30] ), .b(\b[29] ), .c(new_n328), .o1(new_n336));
  xorc02aa1n02x5               g241(.a(\a[31] ), .b(\b[30] ), .out0(new_n337));
  aoi112aa1n03x4               g242(.a(new_n337), .b(new_n336), .c(new_n297), .d(new_n335), .o1(new_n338));
  aoai13aa1n06x5               g243(.a(new_n335), .b(new_n308), .c(new_n196), .d(new_n304), .o1(new_n339));
  inv000aa1n02x5               g244(.a(new_n336), .o1(new_n340));
  inv000aa1d42x5               g245(.a(new_n337), .o1(new_n341));
  tech160nm_fiaoi012aa1n05x5   g246(.a(new_n341), .b(new_n339), .c(new_n340), .o1(new_n342));
  nor002aa1n02x5               g247(.a(new_n342), .b(new_n338), .o1(\s[31] ));
  xnbna2aa1n03x5               g248(.a(new_n110), .b(new_n104), .c(new_n105), .out0(\s[3] ));
  oaoi03aa1n02x5               g249(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n345));
  xorb03aa1n02x5               g250(.a(new_n345), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g251(.a(new_n163), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g252(.a(new_n120), .b(new_n163), .c(new_n119), .o1(new_n348));
  xnrb03aa1n02x5               g253(.a(new_n348), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g254(.a(\a[6] ), .b(\b[5] ), .c(new_n348), .o1(new_n350));
  xorb03aa1n02x5               g255(.a(new_n350), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g256(.a(new_n127), .b(new_n128), .c(new_n350), .o1(new_n352));
  xnrb03aa1n02x5               g257(.a(new_n352), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  norb02aa1n02x5               g258(.a(new_n100), .b(new_n149), .out0(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n354), .b(new_n123), .c(new_n130), .out0(\s[9] ));
endmodule


