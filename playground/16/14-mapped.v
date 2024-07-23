// Benchmark "adder" written by ABC on Wed Jul 17 20:15:35 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n149, new_n150, new_n151, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n158, new_n159, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n179, new_n180, new_n181, new_n182, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n196, new_n197, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n219, new_n220, new_n221, new_n222, new_n223,
    new_n224, new_n225, new_n227, new_n228, new_n229, new_n230, new_n231,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n239, new_n240,
    new_n241, new_n242, new_n243, new_n244, new_n245, new_n246, new_n247,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n260, new_n261, new_n262, new_n263,
    new_n264, new_n265, new_n266, new_n267, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n298, new_n299, new_n300, new_n301, new_n302,
    new_n303, new_n304, new_n305, new_n306, new_n308, new_n309, new_n310,
    new_n311, new_n312, new_n313, new_n314, new_n315, new_n316, new_n318,
    new_n319, new_n320, new_n321, new_n322, new_n323, new_n324, new_n325,
    new_n326, new_n327, new_n328, new_n329, new_n331, new_n332, new_n333,
    new_n334, new_n335, new_n336, new_n337, new_n338, new_n341, new_n342,
    new_n343, new_n344, new_n345, new_n346, new_n347, new_n348, new_n350,
    new_n351, new_n352, new_n353, new_n354, new_n355, new_n356, new_n357,
    new_n360, new_n363, new_n365, new_n366, new_n368;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nand02aa1d04x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  and002aa1n06x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  nor042aa1n04x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nand42aa1n04x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanb02aa1n12x5               g007(.a(new_n101), .b(new_n102), .out0(new_n103));
  nor042aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nand02aa1n03x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  aoi012aa1n02x7               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  oab012aa1n12x5               g012(.a(new_n101), .b(\a[4] ), .c(\b[3] ), .out0(new_n108));
  oaoi13aa1n09x5               g013(.a(new_n100), .b(new_n108), .c(new_n107), .d(new_n103), .o1(new_n109));
  nor002aa1n06x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand02aa1n03x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  norp02aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n112), .b(new_n111), .c(new_n113), .d(new_n110), .out0(new_n114));
  xnrc02aa1n06x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  tech160nm_fixnrc02aa1n04x5   g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n03x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  inv040aa1d32x5               g022(.a(\a[5] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[4] ), .o1(new_n119));
  nand42aa1n03x5               g024(.a(new_n119), .b(new_n118), .o1(new_n120));
  oaoi03aa1n12x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\a[7] ), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\b[6] ), .o1(new_n123));
  aoai13aa1n06x5               g028(.a(new_n111), .b(new_n110), .c(new_n122), .d(new_n123), .o1(new_n124));
  oaib12aa1n09x5               g029(.a(new_n124), .b(new_n114), .c(new_n121), .out0(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n109), .d(new_n117), .o1(new_n127));
  nor042aa1n12x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand02aa1d24x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n12x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g036(.a(new_n128), .o1(new_n132));
  inv000aa1d42x5               g037(.a(new_n129), .o1(new_n133));
  nor042aa1d18x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n08x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  inv000aa1n02x5               g041(.a(new_n136), .o1(new_n137));
  aoi113aa1n03x7               g042(.a(new_n137), .b(new_n133), .c(new_n127), .d(new_n132), .e(new_n99), .o1(new_n138));
  inv000aa1n02x5               g043(.a(new_n104), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(new_n106), .b(new_n105), .o1(new_n140));
  aoai13aa1n12x5               g045(.a(new_n108), .b(new_n103), .c(new_n140), .d(new_n139), .o1(new_n141));
  nor042aa1n02x5               g046(.a(new_n115), .b(new_n116), .o1(new_n142));
  nona23aa1n09x5               g047(.a(new_n141), .b(new_n142), .c(new_n114), .d(new_n100), .out0(new_n143));
  nano23aa1n06x5               g048(.a(new_n113), .b(new_n110), .c(new_n111), .d(new_n112), .out0(new_n144));
  aobi12aa1n06x5               g049(.a(new_n124), .b(new_n144), .c(new_n121), .out0(new_n145));
  nand02aa1n06x5               g050(.a(\b[8] ), .b(\a[9] ), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  aoai13aa1n03x5               g052(.a(new_n99), .b(new_n147), .c(new_n143), .d(new_n145), .o1(new_n148));
  aoai13aa1n06x5               g053(.a(new_n129), .b(new_n128), .c(new_n97), .d(new_n98), .o1(new_n149));
  inv040aa1n03x5               g054(.a(new_n149), .o1(new_n150));
  aoi112aa1n02x5               g055(.a(new_n150), .b(new_n136), .c(new_n148), .d(new_n130), .o1(new_n151));
  norp02aa1n02x5               g056(.a(new_n151), .b(new_n138), .o1(\s[11] ));
  inv000aa1d42x5               g057(.a(new_n134), .o1(new_n153));
  nor042aa1n04x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  nanp02aa1n04x5               g059(.a(\b[11] ), .b(\a[12] ), .o1(new_n155));
  nanb02aa1n02x5               g060(.a(new_n154), .b(new_n155), .out0(new_n156));
  nano22aa1n02x4               g061(.a(new_n138), .b(new_n153), .c(new_n156), .out0(new_n157));
  aoai13aa1n03x5               g062(.a(new_n136), .b(new_n150), .c(new_n148), .d(new_n130), .o1(new_n158));
  tech160nm_fiaoi012aa1n02p5x5 g063(.a(new_n156), .b(new_n158), .c(new_n153), .o1(new_n159));
  norp02aa1n02x5               g064(.a(new_n159), .b(new_n157), .o1(\s[12] ));
  nona23aa1d18x5               g065(.a(new_n155), .b(new_n135), .c(new_n134), .d(new_n154), .out0(new_n161));
  nano32aa1d15x5               g066(.a(new_n161), .b(new_n130), .c(new_n146), .d(new_n99), .out0(new_n162));
  aoai13aa1n03x5               g067(.a(new_n162), .b(new_n125), .c(new_n109), .d(new_n117), .o1(new_n163));
  nano23aa1n06x5               g068(.a(new_n134), .b(new_n154), .c(new_n155), .d(new_n135), .out0(new_n164));
  oaih12aa1n02x5               g069(.a(new_n155), .b(new_n154), .c(new_n134), .o1(new_n165));
  aobi12aa1n06x5               g070(.a(new_n165), .b(new_n164), .c(new_n150), .out0(new_n166));
  nor002aa1n12x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  nand42aa1n03x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n163), .c(new_n166), .out0(\s[13] ));
  inv000aa1d42x5               g075(.a(\a[13] ), .o1(new_n171));
  inv000aa1d42x5               g076(.a(\b[12] ), .o1(new_n172));
  inv000aa1d42x5               g077(.a(new_n162), .o1(new_n173));
  aoai13aa1n03x5               g078(.a(new_n166), .b(new_n173), .c(new_n143), .d(new_n145), .o1(new_n174));
  oaoi03aa1n02x5               g079(.a(new_n171), .b(new_n172), .c(new_n174), .o1(new_n175));
  orn002aa1n02x5               g080(.a(\a[14] ), .b(\b[13] ), .o(new_n176));
  nand42aa1n10x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  xnbna2aa1n03x5               g082(.a(new_n175), .b(new_n177), .c(new_n176), .out0(\s[14] ));
  nor022aa1n16x5               g083(.a(\b[13] ), .b(\a[14] ), .o1(new_n179));
  aoai13aa1n12x5               g084(.a(new_n177), .b(new_n179), .c(new_n171), .d(new_n172), .o1(new_n180));
  nona23aa1n03x5               g085(.a(new_n177), .b(new_n168), .c(new_n167), .d(new_n179), .out0(new_n181));
  aoai13aa1n04x5               g086(.a(new_n180), .b(new_n181), .c(new_n163), .d(new_n166), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g088(.a(\b[14] ), .b(\a[15] ), .o1(new_n184));
  nanp02aa1n04x5               g089(.a(\b[14] ), .b(\a[15] ), .o1(new_n185));
  nanb02aa1n12x5               g090(.a(new_n184), .b(new_n185), .out0(new_n186));
  inv000aa1d42x5               g091(.a(new_n186), .o1(new_n187));
  nor002aa1n06x5               g092(.a(\b[15] ), .b(\a[16] ), .o1(new_n188));
  nand02aa1n04x5               g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  nanb02aa1n18x5               g094(.a(new_n188), .b(new_n189), .out0(new_n190));
  inv000aa1d42x5               g095(.a(new_n190), .o1(new_n191));
  aoi112aa1n03x4               g096(.a(new_n191), .b(new_n184), .c(new_n182), .d(new_n187), .o1(new_n192));
  inv000aa1d42x5               g097(.a(new_n184), .o1(new_n193));
  inv000aa1d42x5               g098(.a(new_n180), .o1(new_n194));
  nano23aa1n03x7               g099(.a(new_n167), .b(new_n179), .c(new_n177), .d(new_n168), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n187), .b(new_n194), .c(new_n174), .d(new_n195), .o1(new_n196));
  aoi012aa1n02x5               g101(.a(new_n190), .b(new_n196), .c(new_n193), .o1(new_n197));
  norp02aa1n02x5               g102(.a(new_n197), .b(new_n192), .o1(\s[16] ));
  nano32aa1n03x7               g103(.a(new_n128), .b(new_n99), .c(new_n129), .d(new_n146), .out0(new_n199));
  nona22aa1n09x5               g104(.a(new_n195), .b(new_n190), .c(new_n186), .out0(new_n200));
  nano22aa1n03x7               g105(.a(new_n200), .b(new_n199), .c(new_n164), .out0(new_n201));
  aoai13aa1n06x5               g106(.a(new_n201), .b(new_n125), .c(new_n109), .d(new_n117), .o1(new_n202));
  oai012aa1n04x7               g107(.a(new_n165), .b(new_n161), .c(new_n149), .o1(new_n203));
  nona23aa1n06x5               g108(.a(new_n189), .b(new_n185), .c(new_n184), .d(new_n188), .out0(new_n204));
  nor042aa1n02x5               g109(.a(new_n204), .b(new_n181), .o1(new_n205));
  oai012aa1n02x5               g110(.a(new_n189), .b(new_n188), .c(new_n184), .o1(new_n206));
  oai012aa1n06x5               g111(.a(new_n206), .b(new_n204), .c(new_n180), .o1(new_n207));
  aoi012aa1n12x5               g112(.a(new_n207), .b(new_n203), .c(new_n205), .o1(new_n208));
  xorc02aa1n02x5               g113(.a(\a[17] ), .b(\b[16] ), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n202), .c(new_n208), .out0(\s[17] ));
  inv000aa1d42x5               g115(.a(\a[17] ), .o1(new_n211));
  inv000aa1d42x5               g116(.a(\b[16] ), .o1(new_n212));
  nanp02aa1n02x5               g117(.a(new_n212), .b(new_n211), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n100), .o1(new_n214));
  norb02aa1n09x5               g119(.a(new_n102), .b(new_n101), .out0(new_n215));
  nanp02aa1n02x5               g120(.a(new_n140), .b(new_n139), .o1(new_n216));
  inv040aa1n03x5               g121(.a(new_n108), .o1(new_n217));
  aoai13aa1n02x7               g122(.a(new_n214), .b(new_n217), .c(new_n216), .d(new_n215), .o1(new_n218));
  nona22aa1n02x4               g123(.a(new_n144), .b(new_n115), .c(new_n116), .out0(new_n219));
  oaih12aa1n02x5               g124(.a(new_n145), .b(new_n218), .c(new_n219), .o1(new_n220));
  oabi12aa1n06x5               g125(.a(new_n207), .b(new_n166), .c(new_n200), .out0(new_n221));
  aoai13aa1n02x5               g126(.a(new_n209), .b(new_n221), .c(new_n220), .d(new_n201), .o1(new_n222));
  nor022aa1n08x5               g127(.a(\b[17] ), .b(\a[18] ), .o1(new_n223));
  nand42aa1n03x5               g128(.a(\b[17] ), .b(\a[18] ), .o1(new_n224));
  nanb02aa1n06x5               g129(.a(new_n223), .b(new_n224), .out0(new_n225));
  xobna2aa1n03x5               g130(.a(new_n225), .b(new_n222), .c(new_n213), .out0(\s[18] ));
  nanp02aa1n02x5               g131(.a(\b[16] ), .b(\a[17] ), .o1(new_n227));
  nano22aa1d15x5               g132(.a(new_n225), .b(new_n213), .c(new_n227), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n224), .b(new_n223), .c(new_n211), .d(new_n212), .o1(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n229), .c(new_n202), .d(new_n208), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g137(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n09x5               g138(.a(\b[18] ), .b(\a[19] ), .o1(new_n234));
  nand02aa1n06x5               g139(.a(\b[18] ), .b(\a[19] ), .o1(new_n235));
  nor002aa1n04x5               g140(.a(\b[19] ), .b(\a[20] ), .o1(new_n236));
  nand42aa1n06x5               g141(.a(\b[19] ), .b(\a[20] ), .o1(new_n237));
  nanb02aa1n02x5               g142(.a(new_n236), .b(new_n237), .out0(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  aoi112aa1n03x4               g144(.a(new_n239), .b(new_n234), .c(new_n231), .d(new_n235), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n234), .o1(new_n241));
  nand02aa1d04x5               g146(.a(new_n162), .b(new_n205), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n208), .b(new_n242), .c(new_n143), .d(new_n145), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n235), .b(new_n234), .out0(new_n244));
  oaoi03aa1n02x5               g149(.a(\a[18] ), .b(\b[17] ), .c(new_n213), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n244), .b(new_n245), .c(new_n243), .d(new_n228), .o1(new_n246));
  aoi012aa1n03x5               g151(.a(new_n238), .b(new_n246), .c(new_n241), .o1(new_n247));
  norp02aa1n03x5               g152(.a(new_n247), .b(new_n240), .o1(\s[20] ));
  nano23aa1d12x5               g153(.a(new_n234), .b(new_n236), .c(new_n237), .d(new_n235), .out0(new_n249));
  nand22aa1n12x5               g154(.a(new_n228), .b(new_n249), .o1(new_n250));
  nona22aa1n02x4               g155(.a(new_n237), .b(new_n236), .c(new_n234), .out0(new_n251));
  aoi022aa1n06x5               g156(.a(new_n249), .b(new_n245), .c(new_n237), .d(new_n251), .o1(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n250), .c(new_n202), .d(new_n208), .o1(new_n253));
  xorb03aa1n02x5               g158(.a(new_n253), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g159(.a(\b[20] ), .b(\a[21] ), .o1(new_n255));
  xnrc02aa1n12x5               g160(.a(\b[20] ), .b(\a[21] ), .out0(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  xnrc02aa1n12x5               g162(.a(\b[21] ), .b(\a[22] ), .out0(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  aoi112aa1n03x4               g164(.a(new_n255), .b(new_n259), .c(new_n253), .d(new_n257), .o1(new_n260));
  inv000aa1n02x5               g165(.a(new_n255), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n250), .o1(new_n262));
  nona23aa1n09x5               g167(.a(new_n237), .b(new_n235), .c(new_n234), .d(new_n236), .out0(new_n263));
  oai012aa1n02x5               g168(.a(new_n237), .b(new_n236), .c(new_n234), .o1(new_n264));
  oai012aa1n09x5               g169(.a(new_n264), .b(new_n263), .c(new_n230), .o1(new_n265));
  aoai13aa1n03x5               g170(.a(new_n257), .b(new_n265), .c(new_n243), .d(new_n262), .o1(new_n266));
  aoi012aa1n03x5               g171(.a(new_n258), .b(new_n266), .c(new_n261), .o1(new_n267));
  norp02aa1n03x5               g172(.a(new_n267), .b(new_n260), .o1(\s[22] ));
  nor042aa1n06x5               g173(.a(new_n258), .b(new_n256), .o1(new_n269));
  nand23aa1d12x5               g174(.a(new_n269), .b(new_n228), .c(new_n249), .o1(new_n270));
  oaoi03aa1n09x5               g175(.a(\a[22] ), .b(\b[21] ), .c(new_n261), .o1(new_n271));
  aoi012aa1d18x5               g176(.a(new_n271), .b(new_n265), .c(new_n269), .o1(new_n272));
  aoai13aa1n04x5               g177(.a(new_n272), .b(new_n270), .c(new_n202), .d(new_n208), .o1(new_n273));
  xorb03aa1n02x5               g178(.a(new_n273), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g179(.a(\b[22] ), .b(\a[23] ), .o1(new_n275));
  nanp02aa1n03x5               g180(.a(\b[22] ), .b(\a[23] ), .o1(new_n276));
  nor042aa1n02x5               g181(.a(\b[23] ), .b(\a[24] ), .o1(new_n277));
  nand02aa1n03x5               g182(.a(\b[23] ), .b(\a[24] ), .o1(new_n278));
  norb02aa1n02x5               g183(.a(new_n278), .b(new_n277), .out0(new_n279));
  aoi112aa1n03x4               g184(.a(new_n275), .b(new_n279), .c(new_n273), .d(new_n276), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n275), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n270), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n272), .o1(new_n283));
  norb02aa1n02x5               g188(.a(new_n276), .b(new_n275), .out0(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n283), .c(new_n243), .d(new_n282), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n279), .o1(new_n286));
  tech160nm_fiaoi012aa1n02p5x5 g191(.a(new_n286), .b(new_n285), .c(new_n281), .o1(new_n287));
  nor042aa1n03x5               g192(.a(new_n287), .b(new_n280), .o1(\s[24] ));
  nona23aa1n12x5               g193(.a(new_n278), .b(new_n276), .c(new_n275), .d(new_n277), .out0(new_n289));
  inv040aa1n03x5               g194(.a(new_n289), .o1(new_n290));
  nano22aa1n02x4               g195(.a(new_n250), .b(new_n290), .c(new_n269), .out0(new_n291));
  inv020aa1n03x5               g196(.a(new_n291), .o1(new_n292));
  oai012aa1n02x5               g197(.a(new_n278), .b(new_n277), .c(new_n275), .o1(new_n293));
  oaib12aa1n09x5               g198(.a(new_n293), .b(new_n289), .c(new_n271), .out0(new_n294));
  aoi013aa1n03x5               g199(.a(new_n294), .b(new_n265), .c(new_n269), .d(new_n290), .o1(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n292), .c(new_n202), .d(new_n208), .o1(new_n296));
  xorb03aa1n02x5               g201(.a(new_n296), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g202(.a(\b[24] ), .b(\a[25] ), .o1(new_n298));
  tech160nm_fixorc02aa1n03p5x5 g203(.a(\a[25] ), .b(\b[24] ), .out0(new_n299));
  xorc02aa1n12x5               g204(.a(\a[26] ), .b(\b[25] ), .out0(new_n300));
  aoi112aa1n03x4               g205(.a(new_n298), .b(new_n300), .c(new_n296), .d(new_n299), .o1(new_n301));
  inv000aa1n03x5               g206(.a(new_n298), .o1(new_n302));
  inv030aa1n03x5               g207(.a(new_n295), .o1(new_n303));
  aoai13aa1n03x5               g208(.a(new_n299), .b(new_n303), .c(new_n243), .d(new_n291), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n300), .o1(new_n305));
  aoi012aa1n03x5               g210(.a(new_n305), .b(new_n304), .c(new_n302), .o1(new_n306));
  nor002aa1n02x5               g211(.a(new_n306), .b(new_n301), .o1(\s[26] ));
  nano32aa1d12x5               g212(.a(new_n270), .b(new_n300), .c(new_n290), .d(new_n299), .out0(new_n308));
  aoai13aa1n06x5               g213(.a(new_n308), .b(new_n221), .c(new_n220), .d(new_n201), .o1(new_n309));
  nano22aa1n03x7               g214(.a(new_n252), .b(new_n269), .c(new_n290), .out0(new_n310));
  and002aa1n02x5               g215(.a(new_n300), .b(new_n299), .o(new_n311));
  oaoi03aa1n12x5               g216(.a(\a[26] ), .b(\b[25] ), .c(new_n302), .o1(new_n312));
  oaoi13aa1n12x5               g217(.a(new_n312), .b(new_n311), .c(new_n310), .d(new_n294), .o1(new_n313));
  nor042aa1n06x5               g218(.a(\b[26] ), .b(\a[27] ), .o1(new_n314));
  nanp02aa1n02x5               g219(.a(\b[26] ), .b(\a[27] ), .o1(new_n315));
  norb02aa1n12x5               g220(.a(new_n315), .b(new_n314), .out0(new_n316));
  xnbna2aa1n03x5               g221(.a(new_n316), .b(new_n309), .c(new_n313), .out0(\s[27] ));
  inv020aa1n02x5               g222(.a(new_n314), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n316), .o1(new_n319));
  tech160nm_fiaoi012aa1n02p5x5 g224(.a(new_n319), .b(new_n309), .c(new_n313), .o1(new_n320));
  xnrc02aa1n02x5               g225(.a(\b[27] ), .b(\a[28] ), .out0(new_n321));
  nano22aa1n03x5               g226(.a(new_n320), .b(new_n318), .c(new_n321), .out0(new_n322));
  nona32aa1n03x5               g227(.a(new_n265), .b(new_n289), .c(new_n258), .d(new_n256), .out0(new_n323));
  aobi12aa1n02x5               g228(.a(new_n293), .b(new_n290), .c(new_n271), .out0(new_n324));
  inv000aa1n02x5               g229(.a(new_n311), .o1(new_n325));
  inv000aa1d42x5               g230(.a(new_n312), .o1(new_n326));
  aoai13aa1n06x5               g231(.a(new_n326), .b(new_n325), .c(new_n323), .d(new_n324), .o1(new_n327));
  aoai13aa1n03x5               g232(.a(new_n316), .b(new_n327), .c(new_n243), .d(new_n308), .o1(new_n328));
  aoi012aa1n03x5               g233(.a(new_n321), .b(new_n328), .c(new_n318), .o1(new_n329));
  nor002aa1n02x5               g234(.a(new_n329), .b(new_n322), .o1(\s[28] ));
  xnrc02aa1n02x5               g235(.a(\b[28] ), .b(\a[29] ), .out0(new_n331));
  nano22aa1n03x7               g236(.a(new_n321), .b(new_n318), .c(new_n315), .out0(new_n332));
  aoai13aa1n03x5               g237(.a(new_n332), .b(new_n327), .c(new_n243), .d(new_n308), .o1(new_n333));
  oao003aa1n02x5               g238(.a(\a[28] ), .b(\b[27] ), .c(new_n318), .carry(new_n334));
  aoi012aa1n03x5               g239(.a(new_n331), .b(new_n333), .c(new_n334), .o1(new_n335));
  inv000aa1d42x5               g240(.a(new_n332), .o1(new_n336));
  tech160nm_fiaoi012aa1n02p5x5 g241(.a(new_n336), .b(new_n309), .c(new_n313), .o1(new_n337));
  nano22aa1n03x5               g242(.a(new_n337), .b(new_n331), .c(new_n334), .out0(new_n338));
  nor002aa1n02x5               g243(.a(new_n335), .b(new_n338), .o1(\s[29] ));
  xorb03aa1n02x5               g244(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g245(.a(new_n331), .b(new_n321), .c(new_n315), .d(new_n318), .out0(new_n341));
  aoai13aa1n03x5               g246(.a(new_n341), .b(new_n327), .c(new_n243), .d(new_n308), .o1(new_n342));
  oao003aa1n02x5               g247(.a(\a[29] ), .b(\b[28] ), .c(new_n334), .carry(new_n343));
  xnrc02aa1n02x5               g248(.a(\b[29] ), .b(\a[30] ), .out0(new_n344));
  aoi012aa1n03x5               g249(.a(new_n344), .b(new_n342), .c(new_n343), .o1(new_n345));
  inv000aa1n02x5               g250(.a(new_n341), .o1(new_n346));
  tech160nm_fiaoi012aa1n02p5x5 g251(.a(new_n346), .b(new_n309), .c(new_n313), .o1(new_n347));
  nano22aa1n03x5               g252(.a(new_n347), .b(new_n343), .c(new_n344), .out0(new_n348));
  nor002aa1n02x5               g253(.a(new_n345), .b(new_n348), .o1(\s[30] ));
  norb03aa1d15x5               g254(.a(new_n332), .b(new_n331), .c(new_n344), .out0(new_n350));
  inv000aa1d42x5               g255(.a(new_n350), .o1(new_n351));
  tech160nm_fiaoi012aa1n04x5   g256(.a(new_n351), .b(new_n309), .c(new_n313), .o1(new_n352));
  oao003aa1n02x5               g257(.a(\a[30] ), .b(\b[29] ), .c(new_n343), .carry(new_n353));
  xnrc02aa1n02x5               g258(.a(\b[30] ), .b(\a[31] ), .out0(new_n354));
  nano22aa1n03x5               g259(.a(new_n352), .b(new_n353), .c(new_n354), .out0(new_n355));
  aoai13aa1n03x5               g260(.a(new_n350), .b(new_n327), .c(new_n243), .d(new_n308), .o1(new_n356));
  aoi012aa1n03x5               g261(.a(new_n354), .b(new_n356), .c(new_n353), .o1(new_n357));
  nor002aa1n02x5               g262(.a(new_n357), .b(new_n355), .o1(\s[31] ));
  xnbna2aa1n03x5               g263(.a(new_n215), .b(new_n140), .c(new_n139), .out0(\s[3] ));
  oaoi03aa1n02x5               g264(.a(\a[3] ), .b(\b[2] ), .c(new_n107), .o1(new_n360));
  xorb03aa1n02x5               g265(.a(new_n360), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnbna2aa1n03x5               g266(.a(new_n116), .b(new_n141), .c(new_n214), .out0(\s[5] ));
  nona22aa1n02x4               g267(.a(new_n141), .b(new_n116), .c(new_n100), .out0(new_n363));
  xobna2aa1n03x5               g268(.a(new_n115), .b(new_n363), .c(new_n120), .out0(\s[6] ));
  oa0022aa1n02x5               g269(.a(\a[6] ), .b(\b[5] ), .c(\a[5] ), .d(\b[4] ), .o(new_n365));
  aoi022aa1n02x5               g270(.a(new_n363), .b(new_n365), .c(\a[6] ), .d(\b[5] ), .o1(new_n366));
  xorb03aa1n02x5               g271(.a(new_n366), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g272(.a(new_n122), .b(new_n123), .c(new_n366), .o1(new_n368));
  xnrb03aa1n02x5               g273(.a(new_n368), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g274(.a(new_n126), .b(new_n143), .c(new_n145), .out0(\s[9] ));
endmodule


