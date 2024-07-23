// Benchmark "adder" written by ABC on Wed Jul 17 16:14:25 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n339, new_n341, new_n343, new_n345, new_n347, new_n348,
    new_n349, new_n350, new_n351, new_n353, new_n354, new_n355;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\b[8] ), .o1(new_n97));
  nanb02aa1n06x5               g002(.a(\a[9] ), .b(new_n97), .out0(new_n98));
  nor002aa1n06x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  inv020aa1n04x5               g004(.a(new_n99), .o1(new_n100));
  nanp02aa1n12x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand02aa1d24x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  aob012aa1n12x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .out0(new_n103));
  nor002aa1d32x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand02aa1n06x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n03x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nor002aa1d32x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nand02aa1d08x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  norb02aa1n03x5               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  nanp03aa1n09x5               g014(.a(new_n103), .b(new_n106), .c(new_n109), .o1(new_n110));
  tech160nm_fioai012aa1n05x5   g015(.a(new_n105), .b(new_n107), .c(new_n104), .o1(new_n111));
  nor002aa1d24x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand02aa1d28x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nor042aa1n09x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nand22aa1n04x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nano23aa1n03x5               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  nor042aa1d18x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nand02aa1d28x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nor042aa1n04x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nanp02aa1n12x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nano23aa1n02x5               g025(.a(new_n117), .b(new_n119), .c(new_n120), .d(new_n118), .out0(new_n121));
  nand02aa1n02x5               g026(.a(new_n121), .b(new_n116), .o1(new_n122));
  nano22aa1n03x7               g027(.a(new_n112), .b(new_n113), .c(new_n120), .out0(new_n123));
  oai012aa1n06x5               g028(.a(new_n118), .b(\b[7] ), .c(\a[8] ), .o1(new_n124));
  oab012aa1n03x5               g029(.a(new_n124), .b(new_n114), .c(new_n117), .out0(new_n125));
  inv040aa1n02x5               g030(.a(new_n112), .o1(new_n126));
  oaoi03aa1n12x5               g031(.a(\a[8] ), .b(\b[7] ), .c(new_n126), .o1(new_n127));
  aoi012aa1n12x5               g032(.a(new_n127), .b(new_n125), .c(new_n123), .o1(new_n128));
  aoai13aa1n12x5               g033(.a(new_n128), .b(new_n122), .c(new_n110), .d(new_n111), .o1(new_n129));
  xorc02aa1n12x5               g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  nanp02aa1n03x5               g035(.a(new_n129), .b(new_n130), .o1(new_n131));
  tech160nm_fixorc02aa1n05x5   g036(.a(\a[10] ), .b(\b[9] ), .out0(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n132), .b(new_n131), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g038(.a(\a[10] ), .o1(new_n134));
  xroi22aa1d04x5               g039(.a(new_n134), .b(\b[9] ), .c(new_n97), .d(\a[9] ), .out0(new_n135));
  oaoi03aa1n12x5               g040(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n136));
  nor042aa1d18x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nand02aa1d24x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norb02aa1n03x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  aoai13aa1n06x5               g044(.a(new_n139), .b(new_n136), .c(new_n129), .d(new_n135), .o1(new_n140));
  aoi112aa1n02x7               g045(.a(new_n139), .b(new_n136), .c(new_n129), .d(new_n135), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(\s[11] ));
  orn002aa1n02x5               g047(.a(\a[11] ), .b(\b[10] ), .o(new_n143));
  nor042aa1n09x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand42aa1d28x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n06x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n146), .b(new_n140), .c(new_n143), .out0(\s[12] ));
  aoi012aa1n02x5               g052(.a(new_n99), .b(new_n101), .c(new_n102), .o1(new_n148));
  nona23aa1n03x5               g053(.a(new_n108), .b(new_n105), .c(new_n104), .d(new_n107), .out0(new_n149));
  oai012aa1n06x5               g054(.a(new_n111), .b(new_n149), .c(new_n148), .o1(new_n150));
  nanb02aa1n09x5               g055(.a(new_n122), .b(new_n150), .out0(new_n151));
  nano23aa1n06x5               g056(.a(new_n137), .b(new_n144), .c(new_n145), .d(new_n138), .out0(new_n152));
  inv030aa1n04x5               g057(.a(new_n152), .o1(new_n153));
  nano22aa1n03x7               g058(.a(new_n153), .b(new_n130), .c(new_n132), .out0(new_n154));
  inv000aa1n02x5               g059(.a(new_n154), .o1(new_n155));
  aoi012aa1n02x7               g060(.a(new_n144), .b(new_n137), .c(new_n145), .o1(new_n156));
  aobi12aa1n02x7               g061(.a(new_n156), .b(new_n152), .c(new_n136), .out0(new_n157));
  aoai13aa1n02x7               g062(.a(new_n157), .b(new_n155), .c(new_n151), .d(new_n128), .o1(new_n158));
  nor042aa1d18x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nand42aa1d28x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  inv040aa1n02x5               g066(.a(new_n157), .o1(new_n162));
  aoi112aa1n03x4               g067(.a(new_n161), .b(new_n162), .c(new_n129), .d(new_n154), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n163), .b(new_n158), .c(new_n161), .o1(\s[13] ));
  orn002aa1n02x5               g069(.a(\a[13] ), .b(\b[12] ), .o(new_n165));
  aoai13aa1n03x5               g070(.a(new_n161), .b(new_n162), .c(new_n129), .d(new_n154), .o1(new_n166));
  nor002aa1d32x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nand42aa1d28x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n166), .c(new_n165), .out0(\s[14] ));
  nano23aa1n09x5               g075(.a(new_n159), .b(new_n167), .c(new_n168), .d(new_n160), .out0(new_n171));
  aoai13aa1n03x5               g076(.a(new_n171), .b(new_n162), .c(new_n129), .d(new_n154), .o1(new_n172));
  aoi012aa1d24x5               g077(.a(new_n167), .b(new_n159), .c(new_n168), .o1(new_n173));
  nor002aa1n10x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nand22aa1n12x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n172), .c(new_n173), .out0(\s[15] ));
  inv020aa1n03x5               g082(.a(new_n173), .o1(new_n178));
  aoai13aa1n03x5               g083(.a(new_n176), .b(new_n178), .c(new_n158), .d(new_n171), .o1(new_n179));
  nor002aa1n03x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nand22aa1n12x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n181), .b(new_n180), .out0(new_n182));
  aoib12aa1n02x5               g087(.a(new_n174), .b(new_n181), .c(new_n180), .out0(new_n183));
  inv000aa1n02x5               g088(.a(new_n174), .o1(new_n184));
  inv000aa1n02x5               g089(.a(new_n176), .o1(new_n185));
  aoai13aa1n03x5               g090(.a(new_n184), .b(new_n185), .c(new_n172), .d(new_n173), .o1(new_n186));
  aoi022aa1n03x5               g091(.a(new_n186), .b(new_n182), .c(new_n179), .d(new_n183), .o1(\s[16] ));
  nano23aa1n06x5               g092(.a(new_n174), .b(new_n180), .c(new_n181), .d(new_n175), .out0(new_n188));
  nand22aa1n03x5               g093(.a(new_n188), .b(new_n171), .o1(new_n189));
  nano32aa1n03x7               g094(.a(new_n189), .b(new_n152), .c(new_n132), .d(new_n130), .out0(new_n190));
  nanp02aa1n02x5               g095(.a(new_n129), .b(new_n190), .o1(new_n191));
  inv020aa1n04x5               g096(.a(new_n190), .o1(new_n192));
  techfinor002aa1n02p5x5       g097(.a(\b[9] ), .b(\a[10] ), .o1(new_n193));
  aoi112aa1n06x5               g098(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n194));
  oai112aa1n03x5               g099(.a(new_n139), .b(new_n146), .c(new_n194), .d(new_n193), .o1(new_n195));
  aoi012aa1n02x5               g100(.a(new_n189), .b(new_n195), .c(new_n156), .o1(new_n196));
  oaoi03aa1n03x5               g101(.a(\a[16] ), .b(\b[15] ), .c(new_n184), .o1(new_n197));
  tech160nm_fiaoi012aa1n02p5x5 g102(.a(new_n197), .b(new_n188), .c(new_n178), .o1(new_n198));
  norb02aa1n03x5               g103(.a(new_n198), .b(new_n196), .out0(new_n199));
  aoai13aa1n04x5               g104(.a(new_n199), .b(new_n192), .c(new_n151), .d(new_n128), .o1(new_n200));
  xorc02aa1n12x5               g105(.a(\a[17] ), .b(\b[16] ), .out0(new_n201));
  norb03aa1n02x5               g106(.a(new_n198), .b(new_n196), .c(new_n201), .out0(new_n202));
  aoi022aa1n03x5               g107(.a(new_n200), .b(new_n201), .c(new_n191), .d(new_n202), .o1(\s[17] ));
  inv000aa1d42x5               g108(.a(\a[17] ), .o1(new_n204));
  nanb02aa1n02x5               g109(.a(\b[16] ), .b(new_n204), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n198), .b(new_n189), .c(new_n195), .d(new_n156), .o1(new_n206));
  aoai13aa1n03x5               g111(.a(new_n201), .b(new_n206), .c(new_n129), .d(new_n190), .o1(new_n207));
  xorc02aa1n12x5               g112(.a(\a[18] ), .b(\b[17] ), .out0(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n208), .b(new_n207), .c(new_n205), .out0(\s[18] ));
  and002aa1n02x5               g114(.a(new_n208), .b(new_n201), .o(new_n210));
  aoai13aa1n03x5               g115(.a(new_n210), .b(new_n206), .c(new_n129), .d(new_n190), .o1(new_n211));
  norp02aa1n03x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  aoi112aa1n09x5               g117(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n213));
  nor002aa1n02x5               g118(.a(new_n213), .b(new_n212), .o1(new_n214));
  xorc02aa1n12x5               g119(.a(\a[19] ), .b(\b[18] ), .out0(new_n215));
  xnbna2aa1n03x5               g120(.a(new_n215), .b(new_n211), .c(new_n214), .out0(\s[19] ));
  xnrc02aa1n02x5               g121(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n02x5               g122(.a(new_n214), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n215), .b(new_n218), .c(new_n200), .d(new_n210), .o1(new_n219));
  xorc02aa1n12x5               g124(.a(\a[20] ), .b(\b[19] ), .out0(new_n220));
  nor002aa1d32x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  norp02aa1n02x5               g126(.a(new_n220), .b(new_n221), .o1(new_n222));
  inv040aa1d30x5               g127(.a(new_n221), .o1(new_n223));
  inv000aa1n02x5               g128(.a(new_n215), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n223), .b(new_n224), .c(new_n211), .d(new_n214), .o1(new_n225));
  aoi022aa1n03x5               g130(.a(new_n225), .b(new_n220), .c(new_n219), .d(new_n222), .o1(\s[20] ));
  nano32aa1n02x4               g131(.a(new_n224), .b(new_n220), .c(new_n201), .d(new_n208), .out0(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n206), .c(new_n129), .d(new_n190), .o1(new_n228));
  oai112aa1n06x5               g133(.a(new_n215), .b(new_n220), .c(new_n213), .d(new_n212), .o1(new_n229));
  oao003aa1n06x5               g134(.a(\a[20] ), .b(\b[19] ), .c(new_n223), .carry(new_n230));
  nand02aa1d06x5               g135(.a(new_n229), .b(new_n230), .o1(new_n231));
  inv000aa1n02x5               g136(.a(new_n231), .o1(new_n232));
  nor002aa1d32x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  nand02aa1d24x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  norb02aa1n03x4               g139(.a(new_n234), .b(new_n233), .out0(new_n235));
  xnbna2aa1n03x5               g140(.a(new_n235), .b(new_n228), .c(new_n232), .out0(\s[21] ));
  aoai13aa1n03x5               g141(.a(new_n235), .b(new_n231), .c(new_n200), .d(new_n227), .o1(new_n237));
  nor042aa1n09x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nand42aa1d28x5               g143(.a(\b[21] ), .b(\a[22] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  aoib12aa1n02x5               g145(.a(new_n233), .b(new_n239), .c(new_n238), .out0(new_n241));
  inv040aa1n06x5               g146(.a(new_n233), .o1(new_n242));
  inv000aa1n02x5               g147(.a(new_n235), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n242), .b(new_n243), .c(new_n228), .d(new_n232), .o1(new_n244));
  aoi022aa1n03x5               g149(.a(new_n244), .b(new_n240), .c(new_n237), .d(new_n241), .o1(\s[22] ));
  inv000aa1d42x5               g150(.a(\a[19] ), .o1(new_n246));
  inv000aa1d42x5               g151(.a(\a[20] ), .o1(new_n247));
  xroi22aa1d04x5               g152(.a(new_n246), .b(\b[18] ), .c(new_n247), .d(\b[19] ), .out0(new_n248));
  nano23aa1d12x5               g153(.a(new_n233), .b(new_n238), .c(new_n239), .d(new_n234), .out0(new_n249));
  inv000aa1n02x5               g154(.a(new_n249), .o1(new_n250));
  nano32aa1n02x4               g155(.a(new_n250), .b(new_n248), .c(new_n208), .d(new_n201), .out0(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n206), .c(new_n129), .d(new_n190), .o1(new_n252));
  oaoi03aa1n03x5               g157(.a(\a[22] ), .b(\b[21] ), .c(new_n242), .o1(new_n253));
  tech160nm_fiaoi012aa1n04x5   g158(.a(new_n253), .b(new_n231), .c(new_n249), .o1(new_n254));
  nand02aa1n02x5               g159(.a(new_n252), .b(new_n254), .o1(new_n255));
  nor002aa1n10x5               g160(.a(\b[22] ), .b(\a[23] ), .o1(new_n256));
  nand42aa1d28x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  norb02aa1n02x7               g162(.a(new_n257), .b(new_n256), .out0(new_n258));
  aoi112aa1n02x5               g163(.a(new_n258), .b(new_n253), .c(new_n231), .d(new_n249), .o1(new_n259));
  aoi022aa1n02x5               g164(.a(new_n255), .b(new_n258), .c(new_n252), .d(new_n259), .o1(\s[23] ));
  inv040aa1n02x5               g165(.a(new_n254), .o1(new_n261));
  aoai13aa1n02x5               g166(.a(new_n258), .b(new_n261), .c(new_n200), .d(new_n251), .o1(new_n262));
  nor022aa1n16x5               g167(.a(\b[23] ), .b(\a[24] ), .o1(new_n263));
  nand42aa1n20x5               g168(.a(\b[23] ), .b(\a[24] ), .o1(new_n264));
  norb02aa1n02x5               g169(.a(new_n264), .b(new_n263), .out0(new_n265));
  aoib12aa1n02x5               g170(.a(new_n256), .b(new_n264), .c(new_n263), .out0(new_n266));
  inv000aa1n02x5               g171(.a(new_n256), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n258), .o1(new_n268));
  aoai13aa1n03x5               g173(.a(new_n267), .b(new_n268), .c(new_n252), .d(new_n254), .o1(new_n269));
  aoi022aa1n03x5               g174(.a(new_n269), .b(new_n265), .c(new_n262), .d(new_n266), .o1(\s[24] ));
  nano23aa1n03x7               g175(.a(new_n256), .b(new_n263), .c(new_n264), .d(new_n257), .out0(new_n271));
  nand22aa1n03x5               g176(.a(new_n271), .b(new_n249), .o1(new_n272));
  nano32aa1n02x5               g177(.a(new_n272), .b(new_n248), .c(new_n208), .d(new_n201), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n206), .c(new_n129), .d(new_n190), .o1(new_n274));
  oaoi03aa1n02x5               g179(.a(\a[24] ), .b(\b[23] ), .c(new_n267), .o1(new_n275));
  tech160nm_fiaoi012aa1n04x5   g180(.a(new_n275), .b(new_n271), .c(new_n253), .o1(new_n276));
  aoai13aa1n04x5               g181(.a(new_n276), .b(new_n272), .c(new_n229), .d(new_n230), .o1(new_n277));
  inv000aa1n02x5               g182(.a(new_n277), .o1(new_n278));
  nanp02aa1n03x5               g183(.a(new_n274), .b(new_n278), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  inv000aa1d42x5               g185(.a(new_n280), .o1(new_n281));
  oai112aa1n02x5               g186(.a(new_n276), .b(new_n281), .c(new_n232), .d(new_n272), .o1(new_n282));
  aboi22aa1n03x5               g187(.a(new_n282), .b(new_n274), .c(new_n279), .d(new_n280), .out0(\s[25] ));
  aoai13aa1n03x5               g188(.a(new_n280), .b(new_n277), .c(new_n200), .d(new_n273), .o1(new_n284));
  tech160nm_fixorc02aa1n03p5x5 g189(.a(\a[26] ), .b(\b[25] ), .out0(new_n285));
  nor042aa1n03x5               g190(.a(\b[24] ), .b(\a[25] ), .o1(new_n286));
  norp02aa1n02x5               g191(.a(new_n285), .b(new_n286), .o1(new_n287));
  inv040aa1n03x5               g192(.a(new_n286), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n281), .c(new_n274), .d(new_n278), .o1(new_n289));
  aoi022aa1n02x7               g194(.a(new_n289), .b(new_n285), .c(new_n284), .d(new_n287), .o1(\s[26] ));
  and002aa1n06x5               g195(.a(new_n285), .b(new_n280), .o(new_n291));
  nano32aa1n03x7               g196(.a(new_n272), .b(new_n291), .c(new_n210), .d(new_n248), .out0(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n206), .c(new_n129), .d(new_n190), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[26] ), .b(\b[25] ), .c(new_n288), .carry(new_n294));
  inv000aa1n02x5               g199(.a(new_n294), .o1(new_n295));
  tech160nm_fiaoi012aa1n05x5   g200(.a(new_n295), .b(new_n277), .c(new_n291), .o1(new_n296));
  nanp02aa1n03x5               g201(.a(new_n293), .b(new_n296), .o1(new_n297));
  xorc02aa1n12x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  aoi112aa1n02x5               g203(.a(new_n298), .b(new_n295), .c(new_n277), .d(new_n291), .o1(new_n299));
  aoi022aa1n02x7               g204(.a(new_n297), .b(new_n298), .c(new_n293), .d(new_n299), .o1(\s[27] ));
  aob012aa1n06x5               g205(.a(new_n294), .b(new_n277), .c(new_n291), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n298), .b(new_n301), .c(new_n200), .d(new_n292), .o1(new_n302));
  xorc02aa1n12x5               g207(.a(\a[28] ), .b(\b[27] ), .out0(new_n303));
  nor042aa1n04x5               g208(.a(\b[26] ), .b(\a[27] ), .o1(new_n304));
  norp02aa1n02x5               g209(.a(new_n303), .b(new_n304), .o1(new_n305));
  inv000aa1n06x5               g210(.a(new_n304), .o1(new_n306));
  inv000aa1d42x5               g211(.a(new_n298), .o1(new_n307));
  aoai13aa1n03x5               g212(.a(new_n306), .b(new_n307), .c(new_n293), .d(new_n296), .o1(new_n308));
  aoi022aa1n03x5               g213(.a(new_n308), .b(new_n303), .c(new_n302), .d(new_n305), .o1(\s[28] ));
  and002aa1n02x5               g214(.a(new_n303), .b(new_n298), .o(new_n310));
  aoai13aa1n02x5               g215(.a(new_n310), .b(new_n301), .c(new_n200), .d(new_n292), .o1(new_n311));
  inv000aa1n02x5               g216(.a(new_n310), .o1(new_n312));
  oaoi03aa1n12x5               g217(.a(\a[28] ), .b(\b[27] ), .c(new_n306), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n313), .o1(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n312), .c(new_n293), .d(new_n296), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  norp02aa1n02x5               g221(.a(new_n313), .b(new_n316), .o1(new_n317));
  aoi022aa1n03x5               g222(.a(new_n315), .b(new_n316), .c(new_n311), .d(new_n317), .o1(\s[29] ));
  xorb03aa1n02x5               g223(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g224(.a(new_n307), .b(new_n303), .c(new_n316), .out0(new_n320));
  aoai13aa1n02x5               g225(.a(new_n320), .b(new_n301), .c(new_n200), .d(new_n292), .o1(new_n321));
  inv000aa1n02x5               g226(.a(new_n320), .o1(new_n322));
  inv000aa1d42x5               g227(.a(\a[29] ), .o1(new_n323));
  inv000aa1d42x5               g228(.a(\b[28] ), .o1(new_n324));
  tech160nm_fioaoi03aa1n03p5x5 g229(.a(new_n323), .b(new_n324), .c(new_n313), .o1(new_n325));
  aoai13aa1n03x5               g230(.a(new_n325), .b(new_n322), .c(new_n293), .d(new_n296), .o1(new_n326));
  xorc02aa1n02x5               g231(.a(\a[30] ), .b(\b[29] ), .out0(new_n327));
  oabi12aa1n02x5               g232(.a(new_n327), .b(\a[29] ), .c(\b[28] ), .out0(new_n328));
  oaoi13aa1n02x5               g233(.a(new_n328), .b(new_n313), .c(new_n323), .d(new_n324), .o1(new_n329));
  aoi022aa1n03x5               g234(.a(new_n326), .b(new_n327), .c(new_n321), .d(new_n329), .o1(\s[30] ));
  nano32aa1n06x5               g235(.a(new_n307), .b(new_n327), .c(new_n303), .d(new_n316), .out0(new_n331));
  aoai13aa1n02x5               g236(.a(new_n331), .b(new_n301), .c(new_n200), .d(new_n292), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[31] ), .b(\b[30] ), .out0(new_n333));
  oao003aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .c(new_n325), .carry(new_n334));
  norb02aa1n02x5               g239(.a(new_n334), .b(new_n333), .out0(new_n335));
  inv000aa1n02x5               g240(.a(new_n331), .o1(new_n336));
  aoai13aa1n03x5               g241(.a(new_n334), .b(new_n336), .c(new_n293), .d(new_n296), .o1(new_n337));
  aoi022aa1n03x5               g242(.a(new_n337), .b(new_n333), .c(new_n332), .d(new_n335), .o1(\s[31] ));
  aoi112aa1n02x5               g243(.a(new_n109), .b(new_n99), .c(new_n101), .d(new_n102), .o1(new_n339));
  aoi012aa1n02x5               g244(.a(new_n339), .b(new_n103), .c(new_n109), .o1(\s[3] ));
  aoi112aa1n02x5               g245(.a(new_n107), .b(new_n106), .c(new_n103), .d(new_n108), .o1(new_n341));
  aoib12aa1n02x5               g246(.a(new_n341), .b(new_n150), .c(new_n104), .out0(\s[4] ));
  norb02aa1n02x5               g247(.a(new_n115), .b(new_n114), .out0(new_n343));
  xnbna2aa1n03x5               g248(.a(new_n343), .b(new_n110), .c(new_n111), .out0(\s[5] ));
  aoi012aa1n02x5               g249(.a(new_n114), .b(new_n150), .c(new_n115), .o1(new_n345));
  xnrb03aa1n02x5               g250(.a(new_n345), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g251(.a(new_n113), .o1(new_n347));
  inv000aa1d42x5               g252(.a(new_n118), .o1(new_n348));
  nona22aa1n02x4               g253(.a(new_n345), .b(new_n348), .c(new_n117), .out0(new_n349));
  nona32aa1n03x5               g254(.a(new_n349), .b(new_n348), .c(new_n347), .d(new_n112), .out0(new_n350));
  aoi022aa1n02x5               g255(.a(new_n349), .b(new_n118), .c(new_n126), .d(new_n113), .o1(new_n351));
  norb02aa1n02x5               g256(.a(new_n350), .b(new_n351), .out0(\s[7] ));
  norb02aa1n02x5               g257(.a(new_n120), .b(new_n119), .out0(new_n353));
  nanp02aa1n02x5               g258(.a(new_n350), .b(new_n126), .o1(new_n354));
  aoib12aa1n02x5               g259(.a(new_n112), .b(new_n120), .c(new_n119), .out0(new_n355));
  aoi022aa1n02x5               g260(.a(new_n354), .b(new_n353), .c(new_n350), .d(new_n355), .o1(\s[8] ));
  xnbna2aa1n03x5               g261(.a(new_n130), .b(new_n151), .c(new_n128), .out0(\s[9] ));
endmodule


