// Benchmark "adder" written by ABC on Thu Jul 18 05:25:54 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n342, new_n343, new_n344,
    new_n345, new_n347, new_n348, new_n349, new_n350, new_n352, new_n353,
    new_n355, new_n356, new_n357, new_n359, new_n361, new_n362, new_n364,
    new_n365, new_n366;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  and002aa1n24x5               g003(.a(\b[8] ), .b(\a[9] ), .o(new_n99));
  inv000aa1d42x5               g004(.a(new_n99), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\a[2] ), .o1(new_n101));
  inv040aa1d32x5               g006(.a(\b[1] ), .o1(new_n102));
  nand02aa1d24x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  oaoi03aa1n12x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  tech160nm_finor002aa1n05x5   g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nand42aa1n06x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nor002aa1n16x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nand42aa1n03x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nona23aa1n09x5               g013(.a(new_n108), .b(new_n106), .c(new_n105), .d(new_n107), .out0(new_n109));
  tech160nm_fiaoi012aa1n03p5x5 g014(.a(new_n105), .b(new_n107), .c(new_n106), .o1(new_n110));
  oai012aa1n18x5               g015(.a(new_n110), .b(new_n109), .c(new_n104), .o1(new_n111));
  norp02aa1n12x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nand02aa1n08x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nand42aa1n03x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nona23aa1n09x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  xnrc02aa1n12x5               g021(.a(\b[7] ), .b(\a[8] ), .out0(new_n117));
  xnrc02aa1n12x5               g022(.a(\b[6] ), .b(\a[7] ), .out0(new_n118));
  nor043aa1n09x5               g023(.a(new_n116), .b(new_n117), .c(new_n118), .o1(new_n119));
  aoi012aa1d18x5               g024(.a(new_n112), .b(new_n114), .c(new_n113), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[8] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[7] ), .o1(new_n122));
  norp02aa1n02x5               g027(.a(\b[6] ), .b(\a[7] ), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(new_n121), .b(new_n122), .c(new_n123), .o1(new_n124));
  oai013aa1d12x5               g029(.a(new_n124), .b(new_n118), .c(new_n117), .d(new_n120), .o1(new_n125));
  aoai13aa1n02x5               g030(.a(new_n100), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n126));
  tech160nm_fixorc02aa1n03p5x5 g031(.a(\a[10] ), .b(\b[9] ), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n03x5               g033(.a(new_n126), .b(new_n98), .o1(new_n129));
  oaoi03aa1n02x5               g034(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n130));
  norp02aa1n24x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand42aa1n06x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  aoai13aa1n06x5               g038(.a(new_n133), .b(new_n130), .c(new_n129), .d(new_n127), .o1(new_n134));
  aoi112aa1n02x5               g039(.a(new_n133), .b(new_n130), .c(new_n129), .d(new_n127), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(\s[11] ));
  inv000aa1d42x5               g041(.a(new_n131), .o1(new_n137));
  nor002aa1n03x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1n06x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  norb03aa1n02x5               g045(.a(new_n139), .b(new_n131), .c(new_n138), .out0(new_n141));
  nanp02aa1n03x5               g046(.a(new_n134), .b(new_n141), .o1(new_n142));
  aoai13aa1n03x5               g047(.a(new_n142), .b(new_n140), .c(new_n134), .d(new_n137), .o1(\s[12] ));
  nor042aa1n09x5               g048(.a(new_n99), .b(new_n97), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  nano32aa1n03x7               g050(.a(new_n145), .b(new_n127), .c(new_n133), .d(new_n140), .out0(new_n146));
  aoai13aa1n06x5               g051(.a(new_n146), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n147));
  oai022aa1n02x5               g052(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n148));
  nano22aa1n03x5               g053(.a(new_n138), .b(new_n132), .c(new_n139), .out0(new_n149));
  aoi012aa1n02x5               g054(.a(new_n131), .b(\a[10] ), .c(\b[9] ), .o1(new_n150));
  nanp03aa1n03x5               g055(.a(new_n149), .b(new_n148), .c(new_n150), .o1(new_n151));
  aoi012aa1n02x5               g056(.a(new_n138), .b(new_n131), .c(new_n139), .o1(new_n152));
  nand02aa1n02x5               g057(.a(new_n151), .b(new_n152), .o1(new_n153));
  nanb02aa1n06x5               g058(.a(new_n153), .b(new_n147), .out0(new_n154));
  xorc02aa1n12x5               g059(.a(\a[13] ), .b(\b[12] ), .out0(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  and003aa1n02x5               g061(.a(new_n151), .b(new_n156), .c(new_n152), .o(new_n157));
  aoi022aa1n02x5               g062(.a(new_n154), .b(new_n155), .c(new_n147), .d(new_n157), .o1(\s[13] ));
  nor042aa1n06x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  xorc02aa1n12x5               g064(.a(\a[14] ), .b(\b[13] ), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  aoai13aa1n02x7               g066(.a(new_n161), .b(new_n159), .c(new_n154), .d(new_n155), .o1(new_n162));
  inv000aa1d42x5               g067(.a(\b[13] ), .o1(new_n163));
  inv000aa1d42x5               g068(.a(\a[14] ), .o1(new_n164));
  aoi012aa1n02x5               g069(.a(new_n159), .b(new_n164), .c(new_n163), .o1(new_n165));
  oaib12aa1n02x5               g070(.a(new_n165), .b(new_n163), .c(\a[14] ), .out0(new_n166));
  aoai13aa1n02x5               g071(.a(new_n162), .b(new_n166), .c(new_n155), .d(new_n154), .o1(\s[14] ));
  and002aa1n02x5               g072(.a(new_n160), .b(new_n155), .o(new_n168));
  oaoi03aa1n12x5               g073(.a(new_n164), .b(new_n163), .c(new_n159), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  nor002aa1n12x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand42aa1n04x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n170), .c(new_n154), .d(new_n168), .o1(new_n174));
  aoi112aa1n02x5               g079(.a(new_n173), .b(new_n170), .c(new_n154), .d(new_n168), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(\s[15] ));
  inv000aa1d42x5               g081(.a(new_n171), .o1(new_n177));
  norp02aa1n04x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand42aa1n04x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  norb03aa1n02x5               g085(.a(new_n179), .b(new_n171), .c(new_n178), .out0(new_n181));
  nanp02aa1n03x5               g086(.a(new_n174), .b(new_n181), .o1(new_n182));
  aoai13aa1n03x5               g087(.a(new_n182), .b(new_n180), .c(new_n177), .d(new_n174), .o1(\s[16] ));
  nano23aa1n02x4               g088(.a(new_n131), .b(new_n138), .c(new_n139), .d(new_n132), .out0(new_n184));
  nano23aa1n03x7               g089(.a(new_n171), .b(new_n178), .c(new_n179), .d(new_n172), .out0(new_n185));
  nand23aa1n02x5               g090(.a(new_n185), .b(new_n155), .c(new_n160), .o1(new_n186));
  nano32aa1n03x7               g091(.a(new_n186), .b(new_n144), .c(new_n184), .d(new_n127), .out0(new_n187));
  aoai13aa1n12x5               g092(.a(new_n187), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n188));
  nona23aa1n09x5               g093(.a(new_n179), .b(new_n172), .c(new_n171), .d(new_n178), .out0(new_n189));
  nano22aa1n03x7               g094(.a(new_n189), .b(new_n155), .c(new_n160), .out0(new_n190));
  aoi012aa1n02x5               g095(.a(new_n178), .b(new_n171), .c(new_n179), .o1(new_n191));
  oai012aa1n02x5               g096(.a(new_n191), .b(new_n189), .c(new_n169), .o1(new_n192));
  aoi012aa1n12x5               g097(.a(new_n192), .b(new_n153), .c(new_n190), .o1(new_n193));
  nand02aa1d10x5               g098(.a(new_n188), .b(new_n193), .o1(new_n194));
  tech160nm_fixorc02aa1n03p5x5 g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  nanb02aa1n02x5               g100(.a(new_n195), .b(new_n191), .out0(new_n196));
  aoi122aa1n02x5               g101(.a(new_n196), .b(new_n170), .c(new_n185), .d(new_n153), .e(new_n190), .o1(new_n197));
  aoi022aa1n02x5               g102(.a(new_n194), .b(new_n195), .c(new_n188), .d(new_n197), .o1(\s[17] ));
  aobi12aa1n02x5               g103(.a(new_n195), .b(new_n188), .c(new_n193), .out0(new_n199));
  nor042aa1n02x5               g104(.a(\b[16] ), .b(\a[17] ), .o1(new_n200));
  aoi012aa1n02x5               g105(.a(new_n200), .b(new_n194), .c(new_n195), .o1(new_n201));
  nor002aa1n04x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nand42aa1n03x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  norb02aa1n06x4               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  nona22aa1n02x4               g109(.a(new_n203), .b(new_n202), .c(new_n200), .out0(new_n205));
  oai022aa1n02x5               g110(.a(new_n201), .b(new_n204), .c(new_n205), .d(new_n199), .o1(\s[18] ));
  and002aa1n02x5               g111(.a(new_n195), .b(new_n204), .o(new_n207));
  aoi012aa1n06x5               g112(.a(new_n202), .b(new_n200), .c(new_n203), .o1(new_n208));
  inv000aa1n09x5               g113(.a(new_n208), .o1(new_n209));
  nor002aa1n10x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n209), .c(new_n194), .d(new_n207), .o1(new_n213));
  aoi112aa1n02x7               g118(.a(new_n212), .b(new_n209), .c(new_n194), .d(new_n207), .o1(new_n214));
  norb02aa1n03x4               g119(.a(new_n213), .b(new_n214), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g121(.a(new_n210), .o1(new_n217));
  nor042aa1n04x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand02aa1d06x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  norb03aa1n02x5               g125(.a(new_n219), .b(new_n210), .c(new_n218), .out0(new_n221));
  nanp02aa1n03x5               g126(.a(new_n213), .b(new_n221), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n222), .b(new_n220), .c(new_n217), .d(new_n213), .o1(\s[20] ));
  nona23aa1n09x5               g128(.a(new_n219), .b(new_n211), .c(new_n210), .d(new_n218), .out0(new_n224));
  nano22aa1n03x7               g129(.a(new_n224), .b(new_n195), .c(new_n204), .out0(new_n225));
  aoi012aa1d18x5               g130(.a(new_n218), .b(new_n210), .c(new_n219), .o1(new_n226));
  tech160nm_fioai012aa1n05x5   g131(.a(new_n226), .b(new_n224), .c(new_n208), .o1(new_n227));
  nor042aa1n09x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  nanp02aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  norb02aa1n02x5               g134(.a(new_n229), .b(new_n228), .out0(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n227), .c(new_n194), .d(new_n225), .o1(new_n231));
  nano23aa1n06x5               g136(.a(new_n210), .b(new_n218), .c(new_n219), .d(new_n211), .out0(new_n232));
  inv000aa1n02x5               g137(.a(new_n226), .o1(new_n233));
  aoi112aa1n02x5               g138(.a(new_n233), .b(new_n230), .c(new_n232), .d(new_n209), .o1(new_n234));
  aobi12aa1n02x5               g139(.a(new_n234), .b(new_n194), .c(new_n225), .out0(new_n235));
  norb02aa1n03x4               g140(.a(new_n231), .b(new_n235), .out0(\s[21] ));
  inv000aa1d42x5               g141(.a(new_n228), .o1(new_n237));
  nor042aa1n03x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  and002aa1n06x5               g143(.a(\b[21] ), .b(\a[22] ), .o(new_n239));
  norp02aa1n02x5               g144(.a(new_n239), .b(new_n238), .o1(new_n240));
  norp03aa1n02x5               g145(.a(new_n239), .b(new_n238), .c(new_n228), .o1(new_n241));
  nanp02aa1n03x5               g146(.a(new_n231), .b(new_n241), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n240), .c(new_n237), .d(new_n231), .o1(\s[22] ));
  nano23aa1n09x5               g148(.a(new_n239), .b(new_n238), .c(new_n237), .d(new_n229), .out0(new_n244));
  nano32aa1n02x4               g149(.a(new_n224), .b(new_n244), .c(new_n195), .d(new_n204), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n244), .b(new_n233), .c(new_n232), .d(new_n209), .o1(new_n246));
  oab012aa1n09x5               g151(.a(new_n238), .b(new_n237), .c(new_n239), .out0(new_n247));
  nanp02aa1n02x5               g152(.a(new_n246), .b(new_n247), .o1(new_n248));
  nor002aa1n12x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  nand42aa1n03x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n250), .b(new_n249), .out0(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n248), .c(new_n194), .d(new_n245), .o1(new_n252));
  inv020aa1n03x5               g157(.a(new_n247), .o1(new_n253));
  nona22aa1n02x4               g158(.a(new_n246), .b(new_n253), .c(new_n251), .out0(new_n254));
  aoi012aa1n02x5               g159(.a(new_n254), .b(new_n194), .c(new_n245), .o1(new_n255));
  norb02aa1n03x4               g160(.a(new_n252), .b(new_n255), .out0(\s[23] ));
  inv000aa1d42x5               g161(.a(new_n249), .o1(new_n257));
  nor042aa1n04x5               g162(.a(\b[23] ), .b(\a[24] ), .o1(new_n258));
  and002aa1n06x5               g163(.a(\b[23] ), .b(\a[24] ), .o(new_n259));
  norp02aa1n02x5               g164(.a(new_n259), .b(new_n258), .o1(new_n260));
  norp03aa1n02x5               g165(.a(new_n259), .b(new_n258), .c(new_n249), .o1(new_n261));
  nanp02aa1n03x5               g166(.a(new_n252), .b(new_n261), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n260), .c(new_n257), .d(new_n252), .o1(\s[24] ));
  inv000aa1n02x5               g168(.a(new_n225), .o1(new_n264));
  nano23aa1d15x5               g169(.a(new_n259), .b(new_n258), .c(new_n257), .d(new_n250), .out0(new_n265));
  nano22aa1n02x5               g170(.a(new_n264), .b(new_n244), .c(new_n265), .out0(new_n266));
  inv000aa1d42x5               g171(.a(new_n265), .o1(new_n267));
  oab012aa1d15x5               g172(.a(new_n258), .b(new_n257), .c(new_n259), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n267), .c(new_n246), .d(new_n247), .o1(new_n269));
  tech160nm_fixorc02aa1n03p5x5 g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n269), .c(new_n194), .d(new_n266), .o1(new_n271));
  nanb02aa1n02x5               g176(.a(new_n270), .b(new_n268), .out0(new_n272));
  aoi122aa1n06x5               g177(.a(new_n272), .b(new_n248), .c(new_n265), .d(new_n194), .e(new_n266), .o1(new_n273));
  norb02aa1n03x4               g178(.a(new_n271), .b(new_n273), .out0(\s[25] ));
  nor042aa1n03x5               g179(.a(\b[24] ), .b(\a[25] ), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  norp02aa1n02x5               g181(.a(\b[25] ), .b(\a[26] ), .o1(new_n277));
  and002aa1n12x5               g182(.a(\b[25] ), .b(\a[26] ), .o(new_n278));
  norp02aa1n06x5               g183(.a(new_n278), .b(new_n277), .o1(new_n279));
  norp03aa1n02x5               g184(.a(new_n278), .b(new_n277), .c(new_n275), .o1(new_n280));
  nanp02aa1n03x5               g185(.a(new_n271), .b(new_n280), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n279), .c(new_n276), .d(new_n271), .o1(\s[26] ));
  and002aa1n06x5               g187(.a(new_n270), .b(new_n279), .o(new_n283));
  nano32aa1n03x7               g188(.a(new_n264), .b(new_n283), .c(new_n244), .d(new_n265), .out0(new_n284));
  aobi12aa1n06x5               g189(.a(new_n284), .b(new_n188), .c(new_n193), .out0(new_n285));
  aoai13aa1n02x5               g190(.a(new_n265), .b(new_n253), .c(new_n227), .d(new_n244), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n283), .o1(new_n287));
  oab012aa1n02x4               g192(.a(new_n277), .b(new_n276), .c(new_n278), .out0(new_n288));
  aoai13aa1n04x5               g193(.a(new_n288), .b(new_n287), .c(new_n286), .d(new_n268), .o1(new_n289));
  xorc02aa1n12x5               g194(.a(\a[27] ), .b(\b[26] ), .out0(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  nanp02aa1n02x5               g196(.a(new_n288), .b(new_n291), .o1(new_n292));
  aoi112aa1n03x4               g197(.a(new_n285), .b(new_n292), .c(new_n269), .d(new_n283), .o1(new_n293));
  oaoi13aa1n02x5               g198(.a(new_n293), .b(new_n290), .c(new_n285), .d(new_n289), .o1(\s[27] ));
  norp02aa1n02x5               g199(.a(\b[26] ), .b(\a[27] ), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  oaih12aa1n02x5               g201(.a(new_n290), .b(new_n289), .c(new_n285), .o1(new_n297));
  nor042aa1n03x5               g202(.a(\b[27] ), .b(\a[28] ), .o1(new_n298));
  and002aa1n02x5               g203(.a(\b[27] ), .b(\a[28] ), .o(new_n299));
  norp02aa1n02x5               g204(.a(new_n299), .b(new_n298), .o1(new_n300));
  nand02aa1d10x5               g205(.a(new_n194), .b(new_n284), .o1(new_n301));
  aobi12aa1n06x5               g206(.a(new_n288), .b(new_n269), .c(new_n283), .out0(new_n302));
  norp03aa1n02x5               g207(.a(new_n299), .b(new_n298), .c(new_n295), .o1(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n291), .c(new_n302), .d(new_n301), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n300), .c(new_n297), .d(new_n296), .o1(\s[28] ));
  inv000aa1d42x5               g210(.a(\a[27] ), .o1(new_n306));
  inv000aa1d42x5               g211(.a(\a[28] ), .o1(new_n307));
  xroi22aa1d04x5               g212(.a(new_n306), .b(\b[26] ), .c(new_n307), .d(\b[27] ), .out0(new_n308));
  oaih12aa1n02x5               g213(.a(new_n308), .b(new_n289), .c(new_n285), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n308), .o1(new_n310));
  aoi112aa1n09x5               g215(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n311));
  oai022aa1n02x5               g216(.a(\a[28] ), .b(\b[27] ), .c(\b[28] ), .d(\a[29] ), .o1(new_n312));
  aoi112aa1n02x5               g217(.a(new_n311), .b(new_n312), .c(\a[29] ), .d(\b[28] ), .o1(new_n313));
  aoai13aa1n03x5               g218(.a(new_n313), .b(new_n310), .c(new_n302), .d(new_n301), .o1(new_n314));
  nor042aa1n04x5               g219(.a(new_n311), .b(new_n298), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n314), .b(new_n316), .c(new_n309), .d(new_n315), .o1(\s[29] ));
  xorb03aa1n02x5               g222(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g223(.a(new_n291), .b(new_n316), .c(new_n300), .out0(new_n319));
  oaih12aa1n02x5               g224(.a(new_n319), .b(new_n289), .c(new_n285), .o1(new_n320));
  tech160nm_fioaoi03aa1n02p5x5 g225(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n321), .o1(new_n322));
  norp02aa1n02x5               g227(.a(\b[29] ), .b(\a[30] ), .o1(new_n323));
  nanp02aa1n02x5               g228(.a(\b[29] ), .b(\a[30] ), .o1(new_n324));
  norb02aa1n02x5               g229(.a(new_n324), .b(new_n323), .out0(new_n325));
  inv000aa1n02x5               g230(.a(new_n319), .o1(new_n326));
  inv000aa1d42x5               g231(.a(\a[29] ), .o1(new_n327));
  obai22aa1n02x7               g232(.a(\b[28] ), .b(new_n327), .c(new_n311), .d(new_n298), .out0(new_n328));
  oai022aa1n02x5               g233(.a(\a[29] ), .b(\b[28] ), .c(\b[29] ), .d(\a[30] ), .o1(new_n329));
  nano22aa1n02x4               g234(.a(new_n329), .b(new_n328), .c(new_n324), .out0(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n326), .c(new_n302), .d(new_n301), .o1(new_n331));
  aoai13aa1n03x5               g236(.a(new_n331), .b(new_n325), .c(new_n320), .d(new_n322), .o1(\s[30] ));
  nano32aa1n02x4               g237(.a(new_n291), .b(new_n325), .c(new_n300), .d(new_n316), .out0(new_n333));
  oaih12aa1n02x5               g238(.a(new_n333), .b(new_n289), .c(new_n285), .o1(new_n334));
  xnrc02aa1n02x5               g239(.a(\b[30] ), .b(\a[31] ), .out0(new_n335));
  inv000aa1d42x5               g240(.a(new_n335), .o1(new_n336));
  inv000aa1n02x5               g241(.a(new_n333), .o1(new_n337));
  aoi112aa1n03x4               g242(.a(new_n323), .b(new_n335), .c(new_n321), .d(new_n325), .o1(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n337), .c(new_n302), .d(new_n301), .o1(new_n339));
  aoi012aa1n02x5               g244(.a(new_n323), .b(new_n321), .c(new_n325), .o1(new_n340));
  aoai13aa1n03x5               g245(.a(new_n339), .b(new_n336), .c(new_n334), .d(new_n340), .o1(\s[31] ));
  aoi022aa1n02x5               g246(.a(new_n102), .b(new_n101), .c(\a[1] ), .d(\b[0] ), .o1(new_n342));
  oaib12aa1n02x5               g247(.a(new_n342), .b(new_n102), .c(\a[2] ), .out0(new_n343));
  norb02aa1n02x5               g248(.a(new_n108), .b(new_n107), .out0(new_n344));
  aboi22aa1n03x5               g249(.a(new_n107), .b(new_n108), .c(new_n101), .d(new_n102), .out0(new_n345));
  aboi22aa1n03x5               g250(.a(new_n104), .b(new_n344), .c(new_n345), .d(new_n343), .out0(\s[3] ));
  inv000aa1d42x5               g251(.a(new_n104), .o1(new_n347));
  oaoi03aa1n02x5               g252(.a(\a[3] ), .b(\b[2] ), .c(new_n104), .o1(new_n348));
  oaib12aa1n02x5               g253(.a(new_n348), .b(new_n105), .c(new_n106), .out0(new_n349));
  nona22aa1n02x4               g254(.a(new_n106), .b(new_n107), .c(new_n105), .out0(new_n350));
  aoai13aa1n02x5               g255(.a(new_n349), .b(new_n350), .c(new_n344), .d(new_n347), .o1(\s[4] ));
  nanb02aa1n02x5               g256(.a(new_n109), .b(new_n347), .out0(new_n352));
  norb02aa1n02x5               g257(.a(new_n115), .b(new_n114), .out0(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n353), .b(new_n352), .c(new_n110), .out0(\s[5] ));
  nanb02aa1n02x5               g259(.a(new_n112), .b(new_n113), .out0(new_n355));
  aoai13aa1n02x5               g260(.a(new_n355), .b(new_n114), .c(new_n111), .d(new_n115), .o1(new_n356));
  nona22aa1n02x4               g261(.a(new_n113), .b(new_n114), .c(new_n112), .out0(new_n357));
  aoai13aa1n02x5               g262(.a(new_n356), .b(new_n357), .c(new_n353), .d(new_n111), .o1(\s[6] ));
  nanb02aa1n02x5               g263(.a(new_n116), .b(new_n111), .out0(new_n359));
  xobna2aa1n03x5               g264(.a(new_n118), .b(new_n359), .c(new_n120), .out0(\s[7] ));
  orn002aa1n02x5               g265(.a(\a[7] ), .b(\b[6] ), .o(new_n361));
  tech160nm_fiao0012aa1n02p5x5 g266(.a(new_n118), .b(new_n359), .c(new_n120), .o(new_n362));
  xobna2aa1n03x5               g267(.a(new_n117), .b(new_n362), .c(new_n361), .out0(\s[8] ));
  aoai13aa1n02x5               g268(.a(new_n145), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n364));
  norb02aa1n02x5               g269(.a(new_n124), .b(new_n145), .out0(new_n365));
  oai013aa1n02x4               g270(.a(new_n365), .b(new_n118), .c(new_n117), .d(new_n120), .o1(new_n366));
  aoai13aa1n02x5               g271(.a(new_n364), .b(new_n366), .c(new_n119), .d(new_n111), .o1(\s[9] ));
endmodule


