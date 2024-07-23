// Benchmark "adder" written by ABC on Thu Jul 18 06:52:03 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n329, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n352,
    new_n353, new_n356, new_n358, new_n359;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor002aa1n12x5               g002(.a(\b[6] ), .b(\a[7] ), .o1(new_n98));
  inv000aa1n02x5               g003(.a(new_n98), .o1(new_n99));
  oaoi03aa1n02x5               g004(.a(\a[8] ), .b(\b[7] ), .c(new_n99), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nor042aa1n02x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  norb02aa1n06x4               g007(.a(new_n101), .b(new_n102), .out0(new_n103));
  orn002aa1n12x5               g008(.a(\a[5] ), .b(\b[4] ), .o(new_n104));
  tech160nm_fioaoi03aa1n03p5x5 g009(.a(\a[6] ), .b(\b[5] ), .c(new_n104), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  norb02aa1n06x4               g011(.a(new_n106), .b(new_n98), .out0(new_n107));
  aoi013aa1n06x4               g012(.a(new_n100), .b(new_n105), .c(new_n107), .d(new_n103), .o1(new_n108));
  nor022aa1n04x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nand02aa1n04x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  nand02aa1n04x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  aoi012aa1n09x5               g016(.a(new_n109), .b(new_n110), .c(new_n111), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  inv040aa1d30x5               g018(.a(\a[4] ), .o1(new_n114));
  inv040aa1d28x5               g019(.a(\b[3] ), .o1(new_n115));
  nand02aa1n04x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  nanp02aa1n04x5               g021(.a(new_n116), .b(new_n113), .o1(new_n117));
  nor042aa1n04x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nand42aa1n02x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  norb02aa1n02x5               g024(.a(new_n119), .b(new_n118), .out0(new_n120));
  nona22aa1n03x5               g025(.a(new_n120), .b(new_n112), .c(new_n117), .out0(new_n121));
  tech160nm_fioaoi03aa1n03p5x5 g026(.a(new_n114), .b(new_n115), .c(new_n118), .o1(new_n122));
  nanb02aa1n02x5               g027(.a(new_n102), .b(new_n101), .out0(new_n123));
  xnrc02aa1n02x5               g028(.a(\b[4] ), .b(\a[5] ), .out0(new_n124));
  nand42aa1n02x5               g029(.a(\b[5] ), .b(\a[6] ), .o1(new_n125));
  norp02aa1n04x5               g030(.a(\b[5] ), .b(\a[6] ), .o1(new_n126));
  nano23aa1n02x4               g031(.a(new_n98), .b(new_n126), .c(new_n106), .d(new_n125), .out0(new_n127));
  nona22aa1n02x4               g032(.a(new_n127), .b(new_n124), .c(new_n123), .out0(new_n128));
  aoai13aa1n06x5               g033(.a(new_n108), .b(new_n128), .c(new_n121), .d(new_n122), .o1(new_n129));
  xorc02aa1n12x5               g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  aoi012aa1n02x5               g035(.a(new_n97), .b(new_n129), .c(new_n130), .o1(new_n131));
  xnrb03aa1n02x5               g036(.a(new_n131), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  xnrc02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .out0(new_n134));
  nor022aa1n08x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nanp03aa1n03x5               g040(.a(new_n105), .b(new_n103), .c(new_n107), .o1(new_n136));
  nanb02aa1n03x5               g041(.a(new_n100), .b(new_n136), .out0(new_n137));
  inv040aa1d32x5               g042(.a(\a[3] ), .o1(new_n138));
  inv000aa1d42x5               g043(.a(\b[2] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(new_n139), .b(new_n138), .o1(new_n140));
  nand42aa1n02x5               g045(.a(new_n140), .b(new_n119), .o1(new_n141));
  oai013aa1n09x5               g046(.a(new_n122), .b(new_n112), .c(new_n117), .d(new_n141), .o1(new_n142));
  nona23aa1n03x5               g047(.a(new_n106), .b(new_n125), .c(new_n98), .d(new_n126), .out0(new_n143));
  nor043aa1n03x5               g048(.a(new_n143), .b(new_n124), .c(new_n123), .o1(new_n144));
  aoai13aa1n03x5               g049(.a(new_n130), .b(new_n137), .c(new_n144), .d(new_n142), .o1(new_n145));
  nona22aa1n02x5               g050(.a(new_n145), .b(new_n97), .c(new_n135), .out0(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n134), .b(new_n146), .c(new_n133), .out0(\s[11] ));
  norp02aa1n02x5               g052(.a(\b[10] ), .b(\a[11] ), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  tech160nm_fixorc02aa1n02p5x5 g054(.a(\a[11] ), .b(\b[10] ), .out0(new_n150));
  nanp03aa1n02x5               g055(.a(new_n146), .b(new_n133), .c(new_n150), .o1(new_n151));
  norp02aa1n02x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  aobi12aa1n02x5               g059(.a(new_n154), .b(new_n151), .c(new_n149), .out0(new_n155));
  aoi113aa1n02x5               g060(.a(new_n148), .b(new_n154), .c(new_n146), .d(new_n150), .e(new_n133), .o1(new_n156));
  nor002aa1n02x5               g061(.a(new_n155), .b(new_n156), .o1(\s[12] ));
  nor003aa1n03x5               g062(.a(new_n112), .b(new_n117), .c(new_n141), .o1(new_n158));
  inv000aa1n02x5               g063(.a(new_n122), .o1(new_n159));
  oai012aa1n06x5               g064(.a(new_n144), .b(new_n158), .c(new_n159), .o1(new_n160));
  nano23aa1n06x5               g065(.a(new_n135), .b(new_n152), .c(new_n153), .d(new_n133), .out0(new_n161));
  nand23aa1n04x5               g066(.a(new_n161), .b(new_n130), .c(new_n150), .o1(new_n162));
  oai022aa1d18x5               g067(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n163));
  aoi022aa1n03x5               g068(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n164));
  oaoi13aa1n06x5               g069(.a(new_n163), .b(new_n164), .c(new_n135), .d(new_n97), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n165), .b(new_n153), .out0(new_n166));
  aoai13aa1n06x5               g071(.a(new_n166), .b(new_n162), .c(new_n160), .d(new_n108), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n24x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  nand02aa1n04x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  aoi012aa1n03x5               g075(.a(new_n169), .b(new_n167), .c(new_n170), .o1(new_n171));
  xnrb03aa1n03x5               g076(.a(new_n171), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv000aa1n02x5               g077(.a(new_n162), .o1(new_n173));
  aoai13aa1n03x5               g078(.a(new_n173), .b(new_n137), .c(new_n142), .d(new_n144), .o1(new_n174));
  norp02aa1n24x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  nand02aa1n08x5               g080(.a(\b[13] ), .b(\a[14] ), .o1(new_n176));
  nona23aa1d18x5               g081(.a(new_n176), .b(new_n170), .c(new_n169), .d(new_n175), .out0(new_n177));
  inv000aa1n02x5               g082(.a(new_n169), .o1(new_n178));
  tech160nm_fioaoi03aa1n03p5x5 g083(.a(\a[14] ), .b(\b[13] ), .c(new_n178), .o1(new_n179));
  inv000aa1d42x5               g084(.a(new_n179), .o1(new_n180));
  aoai13aa1n02x7               g085(.a(new_n180), .b(new_n177), .c(new_n174), .d(new_n166), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  inv020aa1n02x5               g088(.a(new_n183), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n177), .o1(new_n185));
  nand02aa1n06x5               g090(.a(\b[14] ), .b(\a[15] ), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n186), .b(new_n183), .out0(new_n187));
  aoai13aa1n03x5               g092(.a(new_n187), .b(new_n179), .c(new_n167), .d(new_n185), .o1(new_n188));
  nor042aa1n06x5               g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  nand02aa1d28x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  norb02aa1n03x4               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  inv020aa1n03x5               g096(.a(new_n191), .o1(new_n192));
  tech160nm_fiaoi012aa1n02p5x5 g097(.a(new_n192), .b(new_n188), .c(new_n184), .o1(new_n193));
  aoi112aa1n02x5               g098(.a(new_n183), .b(new_n191), .c(new_n181), .d(new_n186), .o1(new_n194));
  nor002aa1n02x5               g099(.a(new_n193), .b(new_n194), .o1(\s[16] ));
  nano32aa1n03x7               g100(.a(new_n192), .b(new_n184), .c(new_n186), .d(new_n153), .out0(new_n196));
  norb02aa1n02x7               g101(.a(new_n176), .b(new_n175), .out0(new_n197));
  nano32aa1n03x7               g102(.a(new_n165), .b(new_n197), .c(new_n178), .d(new_n170), .out0(new_n198));
  oai112aa1n02x5               g103(.a(new_n176), .b(new_n186), .c(new_n175), .d(new_n169), .o1(new_n199));
  nona22aa1n03x5               g104(.a(new_n199), .b(new_n189), .c(new_n183), .out0(new_n200));
  aoi022aa1d24x5               g105(.a(new_n198), .b(new_n196), .c(new_n190), .d(new_n200), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n130), .b(new_n134), .out0(new_n202));
  nano32aa1n02x5               g107(.a(new_n177), .b(new_n191), .c(new_n184), .d(new_n186), .out0(new_n203));
  nanp03aa1n02x5               g108(.a(new_n203), .b(new_n202), .c(new_n161), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n201), .b(new_n204), .c(new_n160), .d(new_n108), .o1(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1n09x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  inv000aa1n02x5               g112(.a(new_n207), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n201), .o1(new_n209));
  nano23aa1n06x5               g114(.a(new_n183), .b(new_n189), .c(new_n190), .d(new_n186), .out0(new_n210));
  nano22aa1d15x5               g115(.a(new_n162), .b(new_n185), .c(new_n210), .out0(new_n211));
  xorc02aa1n12x5               g116(.a(\a[17] ), .b(\b[16] ), .out0(new_n212));
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n209), .c(new_n129), .d(new_n211), .o1(new_n213));
  nor022aa1n08x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  nand42aa1n02x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  nanb02aa1n03x5               g120(.a(new_n214), .b(new_n215), .out0(new_n216));
  xobna2aa1n03x5               g121(.a(new_n216), .b(new_n213), .c(new_n208), .out0(\s[18] ));
  aoai13aa1n06x5               g122(.a(new_n211), .b(new_n137), .c(new_n144), .d(new_n142), .o1(new_n218));
  inv040aa1d30x5               g123(.a(\a[17] ), .o1(new_n219));
  inv040aa1d32x5               g124(.a(\a[18] ), .o1(new_n220));
  xroi22aa1d06x4               g125(.a(new_n219), .b(\b[16] ), .c(new_n220), .d(\b[17] ), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  oaoi03aa1n12x5               g127(.a(\a[18] ), .b(\b[17] ), .c(new_n208), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n222), .c(new_n218), .d(new_n201), .o1(new_n225));
  xorb03aa1n02x5               g130(.a(new_n225), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g131(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g132(.a(\b[18] ), .b(\a[19] ), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  nand02aa1d06x5               g134(.a(\b[18] ), .b(\a[19] ), .o1(new_n230));
  nanb02aa1n12x5               g135(.a(new_n228), .b(new_n230), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n232), .b(new_n223), .c(new_n205), .d(new_n221), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[19] ), .b(\a[20] ), .out0(new_n234));
  aoi012aa1n03x5               g139(.a(new_n234), .b(new_n233), .c(new_n229), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n234), .o1(new_n236));
  aoi112aa1n02x5               g141(.a(new_n228), .b(new_n236), .c(new_n225), .d(new_n230), .o1(new_n237));
  nor002aa1n02x5               g142(.a(new_n235), .b(new_n237), .o1(\s[20] ));
  nona23aa1d18x5               g143(.a(new_n236), .b(new_n212), .c(new_n216), .d(new_n231), .out0(new_n239));
  oab012aa1n09x5               g144(.a(new_n228), .b(\a[20] ), .c(\b[19] ), .out0(new_n240));
  oai112aa1n06x5               g145(.a(new_n215), .b(new_n230), .c(new_n214), .d(new_n207), .o1(new_n241));
  aoi022aa1d18x5               g146(.a(new_n241), .b(new_n240), .c(\a[20] ), .d(\b[19] ), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoai13aa1n04x5               g148(.a(new_n243), .b(new_n239), .c(new_n218), .d(new_n201), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n16x5               g150(.a(\b[20] ), .b(\a[21] ), .o1(new_n246));
  inv040aa1n09x5               g151(.a(new_n246), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n239), .o1(new_n248));
  tech160nm_finand02aa1n03p5x5 g153(.a(\b[20] ), .b(\a[21] ), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n246), .out0(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n242), .c(new_n205), .d(new_n248), .o1(new_n251));
  inv040aa1d32x5               g156(.a(\a[22] ), .o1(new_n252));
  inv040aa1d28x5               g157(.a(\b[21] ), .o1(new_n253));
  nand42aa1n16x5               g158(.a(new_n253), .b(new_n252), .o1(new_n254));
  nanp02aa1n12x5               g159(.a(\b[21] ), .b(\a[22] ), .o1(new_n255));
  nand22aa1n12x5               g160(.a(new_n254), .b(new_n255), .o1(new_n256));
  aoi012aa1n03x5               g161(.a(new_n256), .b(new_n251), .c(new_n247), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n256), .o1(new_n258));
  aoi112aa1n03x4               g163(.a(new_n246), .b(new_n258), .c(new_n244), .d(new_n250), .o1(new_n259));
  norp02aa1n03x5               g164(.a(new_n257), .b(new_n259), .o1(\s[22] ));
  nano22aa1n03x7               g165(.a(new_n256), .b(new_n247), .c(new_n249), .out0(new_n261));
  nona23aa1d18x5               g166(.a(new_n221), .b(new_n261), .c(new_n234), .d(new_n231), .out0(new_n262));
  nanp02aa1n06x5               g167(.a(new_n241), .b(new_n240), .o1(new_n263));
  nand42aa1n03x5               g168(.a(\b[19] ), .b(\a[20] ), .o1(new_n264));
  nano32aa1n03x7               g169(.a(new_n256), .b(new_n247), .c(new_n249), .d(new_n264), .out0(new_n265));
  oaoi03aa1n09x5               g170(.a(new_n252), .b(new_n253), .c(new_n246), .o1(new_n266));
  inv000aa1n02x5               g171(.a(new_n266), .o1(new_n267));
  aoi012aa1n09x5               g172(.a(new_n267), .b(new_n263), .c(new_n265), .o1(new_n268));
  aoai13aa1n04x5               g173(.a(new_n268), .b(new_n262), .c(new_n218), .d(new_n201), .o1(new_n269));
  xorb03aa1n02x5               g174(.a(new_n269), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g175(.a(\b[22] ), .b(\a[23] ), .o1(new_n271));
  inv000aa1n02x5               g176(.a(new_n271), .o1(new_n272));
  inv040aa1n03x5               g177(.a(new_n262), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n268), .o1(new_n274));
  xorc02aa1n12x5               g179(.a(\a[23] ), .b(\b[22] ), .out0(new_n275));
  aoai13aa1n03x5               g180(.a(new_n275), .b(new_n274), .c(new_n205), .d(new_n273), .o1(new_n276));
  xorc02aa1n12x5               g181(.a(\a[24] ), .b(\b[23] ), .out0(new_n277));
  aobi12aa1n03x5               g182(.a(new_n277), .b(new_n276), .c(new_n272), .out0(new_n278));
  aoi112aa1n03x4               g183(.a(new_n271), .b(new_n277), .c(new_n269), .d(new_n275), .o1(new_n279));
  norp02aa1n03x5               g184(.a(new_n278), .b(new_n279), .o1(\s[24] ));
  nano32aa1n06x5               g185(.a(new_n239), .b(new_n277), .c(new_n261), .d(new_n275), .out0(new_n281));
  inv000aa1n02x5               g186(.a(new_n281), .o1(new_n282));
  inv000aa1n02x5               g187(.a(new_n240), .o1(new_n283));
  norp02aa1n02x5               g188(.a(new_n214), .b(new_n207), .o1(new_n284));
  nano22aa1n03x7               g189(.a(new_n284), .b(new_n215), .c(new_n230), .out0(new_n285));
  oai012aa1n06x5               g190(.a(new_n265), .b(new_n285), .c(new_n283), .o1(new_n286));
  nanp02aa1n04x5               g191(.a(new_n277), .b(new_n275), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[24] ), .b(\b[23] ), .c(new_n272), .carry(new_n288));
  aoai13aa1n12x5               g193(.a(new_n288), .b(new_n287), .c(new_n286), .d(new_n266), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n289), .o1(new_n290));
  aoai13aa1n06x5               g195(.a(new_n290), .b(new_n282), .c(new_n218), .d(new_n201), .o1(new_n291));
  xorb03aa1n02x5               g196(.a(new_n291), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g197(.a(\b[24] ), .b(\a[25] ), .o1(new_n293));
  inv000aa1n02x5               g198(.a(new_n293), .o1(new_n294));
  xorc02aa1n03x5               g199(.a(\a[25] ), .b(\b[24] ), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n289), .c(new_n205), .d(new_n281), .o1(new_n296));
  xorc02aa1n03x5               g201(.a(\a[26] ), .b(\b[25] ), .out0(new_n297));
  aobi12aa1n03x5               g202(.a(new_n297), .b(new_n296), .c(new_n294), .out0(new_n298));
  aoi112aa1n03x4               g203(.a(new_n293), .b(new_n297), .c(new_n291), .d(new_n295), .o1(new_n299));
  nor002aa1n02x5               g204(.a(new_n298), .b(new_n299), .o1(\s[26] ));
  inv000aa1n02x5               g205(.a(new_n287), .o1(new_n301));
  and002aa1n02x7               g206(.a(new_n297), .b(new_n295), .o(new_n302));
  nano22aa1d15x5               g207(.a(new_n262), .b(new_n301), .c(new_n302), .out0(new_n303));
  aoai13aa1n06x5               g208(.a(new_n303), .b(new_n209), .c(new_n129), .d(new_n211), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[26] ), .b(\b[25] ), .c(new_n294), .carry(new_n305));
  inv000aa1d42x5               g210(.a(new_n305), .o1(new_n306));
  tech160nm_fiaoi012aa1n03p5x5 g211(.a(new_n306), .b(new_n289), .c(new_n302), .o1(new_n307));
  xorc02aa1n12x5               g212(.a(\a[27] ), .b(\b[26] ), .out0(new_n308));
  xnbna2aa1n03x5               g213(.a(new_n308), .b(new_n304), .c(new_n307), .out0(\s[27] ));
  norp02aa1n02x5               g214(.a(\b[26] ), .b(\a[27] ), .o1(new_n310));
  inv040aa1n03x5               g215(.a(new_n310), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n301), .b(new_n267), .c(new_n263), .d(new_n265), .o1(new_n312));
  inv000aa1n02x5               g217(.a(new_n302), .o1(new_n313));
  aoai13aa1n06x5               g218(.a(new_n305), .b(new_n313), .c(new_n312), .d(new_n288), .o1(new_n314));
  aoai13aa1n03x5               g219(.a(new_n308), .b(new_n314), .c(new_n205), .d(new_n303), .o1(new_n315));
  xnrc02aa1n12x5               g220(.a(\b[27] ), .b(\a[28] ), .out0(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n316), .b(new_n315), .c(new_n311), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n308), .o1(new_n318));
  tech160nm_fiaoi012aa1n02p5x5 g223(.a(new_n318), .b(new_n304), .c(new_n307), .o1(new_n319));
  nano22aa1n03x5               g224(.a(new_n319), .b(new_n311), .c(new_n316), .out0(new_n320));
  norp02aa1n03x5               g225(.a(new_n317), .b(new_n320), .o1(\s[28] ));
  norb02aa1n02x5               g226(.a(new_n308), .b(new_n316), .out0(new_n322));
  aoai13aa1n03x5               g227(.a(new_n322), .b(new_n314), .c(new_n205), .d(new_n303), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[28] ), .b(\b[27] ), .c(new_n311), .carry(new_n324));
  xnrc02aa1n02x5               g229(.a(\b[28] ), .b(\a[29] ), .out0(new_n325));
  tech160nm_fiaoi012aa1n02p5x5 g230(.a(new_n325), .b(new_n323), .c(new_n324), .o1(new_n326));
  inv000aa1n02x5               g231(.a(new_n322), .o1(new_n327));
  tech160nm_fiaoi012aa1n02p5x5 g232(.a(new_n327), .b(new_n304), .c(new_n307), .o1(new_n328));
  nano22aa1n03x5               g233(.a(new_n328), .b(new_n324), .c(new_n325), .out0(new_n329));
  norp02aa1n03x5               g234(.a(new_n326), .b(new_n329), .o1(\s[29] ));
  xorb03aa1n02x5               g235(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n03x5               g236(.a(new_n308), .b(new_n325), .c(new_n316), .out0(new_n332));
  aoai13aa1n03x5               g237(.a(new_n332), .b(new_n314), .c(new_n205), .d(new_n303), .o1(new_n333));
  oao003aa1n02x5               g238(.a(\a[29] ), .b(\b[28] ), .c(new_n324), .carry(new_n334));
  xnrc02aa1n02x5               g239(.a(\b[29] ), .b(\a[30] ), .out0(new_n335));
  tech160nm_fiaoi012aa1n02p5x5 g240(.a(new_n335), .b(new_n333), .c(new_n334), .o1(new_n336));
  inv000aa1d42x5               g241(.a(new_n332), .o1(new_n337));
  tech160nm_fiaoi012aa1n02p5x5 g242(.a(new_n337), .b(new_n304), .c(new_n307), .o1(new_n338));
  nano22aa1n03x5               g243(.a(new_n338), .b(new_n334), .c(new_n335), .out0(new_n339));
  norp02aa1n03x5               g244(.a(new_n336), .b(new_n339), .o1(\s[30] ));
  nanb02aa1n02x5               g245(.a(\b[30] ), .b(\a[31] ), .out0(new_n341));
  nanb02aa1n02x5               g246(.a(\a[31] ), .b(\b[30] ), .out0(new_n342));
  norb02aa1n02x5               g247(.a(new_n332), .b(new_n335), .out0(new_n343));
  aoai13aa1n06x5               g248(.a(new_n343), .b(new_n314), .c(new_n205), .d(new_n303), .o1(new_n344));
  oao003aa1n02x5               g249(.a(\a[30] ), .b(\b[29] ), .c(new_n334), .carry(new_n345));
  aoi022aa1n02x7               g250(.a(new_n344), .b(new_n345), .c(new_n342), .d(new_n341), .o1(new_n346));
  nanp02aa1n02x5               g251(.a(new_n342), .b(new_n341), .o1(new_n347));
  inv000aa1n02x5               g252(.a(new_n345), .o1(new_n348));
  nona22aa1n02x5               g253(.a(new_n344), .b(new_n348), .c(new_n347), .out0(new_n349));
  norb02aa1n03x4               g254(.a(new_n349), .b(new_n346), .out0(\s[31] ));
  xnbna2aa1n03x5               g255(.a(new_n112), .b(new_n119), .c(new_n140), .out0(\s[3] ));
  aoai13aa1n02x5               g256(.a(new_n120), .b(new_n109), .c(new_n111), .d(new_n110), .o1(new_n352));
  aoi022aa1n02x5               g257(.a(new_n116), .b(new_n113), .c(new_n138), .d(new_n139), .o1(new_n353));
  aoi022aa1n02x5               g258(.a(new_n142), .b(new_n116), .c(new_n353), .d(new_n352), .o1(\s[4] ));
  xorb03aa1n02x5               g259(.a(new_n142), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoai13aa1n03x5               g260(.a(new_n104), .b(new_n124), .c(new_n121), .d(new_n122), .o1(new_n356));
  xorb03aa1n02x5               g261(.a(new_n356), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oai112aa1n03x5               g262(.a(new_n125), .b(new_n107), .c(new_n356), .d(new_n126), .o1(new_n358));
  oaoi13aa1n02x5               g263(.a(new_n107), .b(new_n125), .c(new_n356), .d(new_n126), .o1(new_n359));
  norb02aa1n02x5               g264(.a(new_n358), .b(new_n359), .out0(\s[7] ));
  xnbna2aa1n03x5               g265(.a(new_n103), .b(new_n358), .c(new_n99), .out0(\s[8] ));
  xnbna2aa1n03x5               g266(.a(new_n130), .b(new_n160), .c(new_n108), .out0(\s[9] ));
endmodule


