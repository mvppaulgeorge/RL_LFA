// Benchmark "adder" written by ABC on Thu Jul 18 04:55:10 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n308, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n327, new_n329, new_n330, new_n333,
    new_n335, new_n336, new_n338, new_n339;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nanp02aa1n04x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand42aa1n02x5               g002(.a(\b[2] ), .b(\a[3] ), .o1(new_n98));
  nor042aa1d18x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nanb03aa1n03x5               g004(.a(new_n99), .b(new_n97), .c(new_n98), .out0(new_n100));
  oai112aa1n06x5               g005(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n101));
  nanp02aa1n06x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor042aa1n09x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  inv000aa1n02x5               g008(.a(new_n103), .o1(new_n104));
  nanp03aa1n06x5               g009(.a(new_n101), .b(new_n104), .c(new_n102), .o1(new_n105));
  tech160nm_fioai012aa1n03p5x5 g010(.a(new_n102), .b(new_n103), .c(new_n99), .o1(new_n106));
  oai012aa1n06x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  nanp02aa1n04x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nor042aa1n03x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  norb02aa1n02x5               g014(.a(new_n108), .b(new_n109), .out0(new_n110));
  nor002aa1d32x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  norb02aa1n02x7               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nor022aa1n06x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand02aa1d06x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nor002aa1d24x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand22aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nona23aa1n03x5               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  nano22aa1n03x7               g023(.a(new_n118), .b(new_n110), .c(new_n113), .out0(new_n119));
  nano23aa1n06x5               g024(.a(new_n114), .b(new_n116), .c(new_n117), .d(new_n115), .out0(new_n120));
  inv000aa1d42x5               g025(.a(new_n111), .o1(new_n121));
  oaoi03aa1n12x5               g026(.a(\a[6] ), .b(\b[5] ), .c(new_n121), .o1(new_n122));
  nona22aa1n02x4               g027(.a(new_n115), .b(new_n116), .c(new_n114), .out0(new_n123));
  aoi022aa1n12x5               g028(.a(new_n120), .b(new_n122), .c(new_n115), .d(new_n123), .o1(new_n124));
  aobi12aa1n03x7               g029(.a(new_n124), .b(new_n119), .c(new_n107), .out0(new_n125));
  tech160nm_fioaoi03aa1n03p5x5 g030(.a(\a[9] ), .b(\b[8] ), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d32x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nanp02aa1n12x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nor002aa1d24x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  aoi012aa1n02x5               g035(.a(new_n130), .b(\a[10] ), .c(\b[9] ), .o1(new_n131));
  oai112aa1n04x5               g036(.a(new_n129), .b(new_n131), .c(new_n126), .d(new_n128), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n130), .b(new_n129), .out0(new_n133));
  nanp02aa1n04x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  oai012aa1n02x5               g039(.a(new_n134), .b(new_n126), .c(new_n128), .o1(new_n135));
  aobi12aa1n02x7               g040(.a(new_n132), .b(new_n135), .c(new_n133), .out0(\s[11] ));
  inv000aa1n04x5               g041(.a(new_n130), .o1(new_n137));
  nor002aa1n16x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand02aa1d08x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n06x4               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  norb03aa1n02x5               g045(.a(new_n139), .b(new_n130), .c(new_n138), .out0(new_n141));
  tech160nm_finand02aa1n03p5x5 g046(.a(new_n132), .b(new_n141), .o1(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n140), .c(new_n137), .d(new_n132), .o1(\s[12] ));
  nand02aa1d04x5               g048(.a(new_n107), .b(new_n119), .o1(new_n144));
  norb02aa1n02x7               g049(.a(new_n134), .b(new_n128), .out0(new_n145));
  xnrc02aa1n02x5               g050(.a(\b[8] ), .b(\a[9] ), .out0(new_n146));
  nona23aa1n03x5               g051(.a(new_n140), .b(new_n145), .c(new_n146), .d(new_n133), .out0(new_n147));
  nanb03aa1n12x5               g052(.a(new_n138), .b(new_n139), .c(new_n129), .out0(new_n148));
  norp02aa1n12x5               g053(.a(\b[8] ), .b(\a[9] ), .o1(new_n149));
  oai112aa1n06x5               g054(.a(new_n137), .b(new_n134), .c(new_n149), .d(new_n128), .o1(new_n150));
  tech160nm_fioai012aa1n03p5x5 g055(.a(new_n139), .b(new_n138), .c(new_n130), .o1(new_n151));
  oai012aa1n18x5               g056(.a(new_n151), .b(new_n150), .c(new_n148), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n147), .c(new_n144), .d(new_n124), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n12x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  inv000aa1n02x5               g061(.a(new_n156), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp03aa1n02x5               g063(.a(new_n154), .b(new_n157), .c(new_n158), .o1(new_n159));
  nor002aa1n12x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand02aa1n08x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  norb02aa1n06x4               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  nona23aa1n02x4               g067(.a(new_n159), .b(new_n161), .c(new_n160), .d(new_n156), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n162), .c(new_n157), .d(new_n159), .o1(\s[14] ));
  nano23aa1n06x5               g069(.a(new_n156), .b(new_n160), .c(new_n161), .d(new_n158), .out0(new_n165));
  oaih12aa1n12x5               g070(.a(new_n161), .b(new_n160), .c(new_n156), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  nor002aa1d32x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand42aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  aoai13aa1n03x5               g075(.a(new_n170), .b(new_n167), .c(new_n154), .d(new_n165), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(new_n170), .b(new_n167), .c(new_n154), .d(new_n165), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(\s[15] ));
  inv000aa1d42x5               g078(.a(new_n168), .o1(new_n174));
  nor042aa1n03x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nand02aa1d08x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  inv000aa1d42x5               g082(.a(new_n176), .o1(new_n178));
  oai022aa1n02x5               g083(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n179));
  nona22aa1n02x4               g084(.a(new_n171), .b(new_n178), .c(new_n179), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n177), .c(new_n174), .d(new_n171), .o1(\s[16] ));
  nano23aa1n06x5               g086(.a(new_n146), .b(new_n133), .c(new_n145), .d(new_n140), .out0(new_n182));
  nona23aa1n12x5               g087(.a(new_n176), .b(new_n169), .c(new_n168), .d(new_n175), .out0(new_n183));
  nano32aa1n03x7               g088(.a(new_n183), .b(new_n162), .c(new_n158), .d(new_n157), .out0(new_n184));
  nand02aa1n02x5               g089(.a(new_n182), .b(new_n184), .o1(new_n185));
  obai22aa1n03x5               g090(.a(new_n179), .b(new_n178), .c(new_n183), .d(new_n166), .out0(new_n186));
  aoi012aa1n09x5               g091(.a(new_n186), .b(new_n152), .c(new_n184), .o1(new_n187));
  aoai13aa1n12x5               g092(.a(new_n187), .b(new_n185), .c(new_n144), .d(new_n124), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1n06x5               g094(.a(\b[16] ), .b(\a[17] ), .o1(new_n190));
  inv040aa1n02x5               g095(.a(new_n190), .o1(new_n191));
  nano22aa1n02x4               g096(.a(new_n99), .b(new_n97), .c(new_n98), .out0(new_n192));
  norb02aa1n02x5               g097(.a(new_n102), .b(new_n103), .out0(new_n193));
  nanp03aa1n02x5               g098(.a(new_n192), .b(new_n101), .c(new_n193), .o1(new_n194));
  nano23aa1n02x4               g099(.a(new_n111), .b(new_n109), .c(new_n112), .d(new_n108), .out0(new_n195));
  nanp02aa1n02x5               g100(.a(new_n120), .b(new_n195), .o1(new_n196));
  aoai13aa1n06x5               g101(.a(new_n124), .b(new_n196), .c(new_n194), .d(new_n106), .o1(new_n197));
  nano23aa1n03x7               g102(.a(new_n168), .b(new_n175), .c(new_n176), .d(new_n169), .out0(new_n198));
  nanp02aa1n02x5               g103(.a(new_n198), .b(new_n165), .o1(new_n199));
  nor042aa1n02x5               g104(.a(new_n147), .b(new_n199), .o1(new_n200));
  nano22aa1n02x4               g105(.a(new_n138), .b(new_n129), .c(new_n139), .out0(new_n201));
  oai112aa1n02x5               g106(.a(new_n201), .b(new_n131), .c(new_n149), .d(new_n128), .o1(new_n202));
  aoi022aa1n02x5               g107(.a(new_n198), .b(new_n167), .c(new_n176), .d(new_n179), .o1(new_n203));
  aoai13aa1n02x7               g108(.a(new_n203), .b(new_n199), .c(new_n202), .d(new_n151), .o1(new_n204));
  tech160nm_fixorc02aa1n04x5   g109(.a(\a[17] ), .b(\b[16] ), .out0(new_n205));
  aoai13aa1n02x5               g110(.a(new_n205), .b(new_n204), .c(new_n197), .d(new_n200), .o1(new_n206));
  xorc02aa1n03x5               g111(.a(\a[18] ), .b(\b[17] ), .out0(new_n207));
  and002aa1n02x5               g112(.a(\b[17] ), .b(\a[18] ), .o(new_n208));
  oai022aa1n02x5               g113(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n209));
  nona22aa1n02x4               g114(.a(new_n206), .b(new_n208), .c(new_n209), .out0(new_n210));
  aoai13aa1n02x5               g115(.a(new_n210), .b(new_n207), .c(new_n191), .d(new_n206), .o1(\s[18] ));
  and002aa1n02x5               g116(.a(new_n207), .b(new_n205), .o(new_n212));
  tech160nm_fioaoi03aa1n04x5   g117(.a(\a[18] ), .b(\b[17] ), .c(new_n191), .o1(new_n213));
  nor042aa1n06x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nand22aa1n03x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n213), .c(new_n188), .d(new_n212), .o1(new_n217));
  aoi112aa1n02x5               g122(.a(new_n216), .b(new_n213), .c(new_n188), .d(new_n212), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n217), .b(new_n218), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n02x5               g125(.a(new_n214), .o1(new_n221));
  nor042aa1n02x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  nand02aa1n03x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  nona23aa1n03x5               g129(.a(new_n217), .b(new_n223), .c(new_n222), .d(new_n214), .out0(new_n225));
  aoai13aa1n03x5               g130(.a(new_n225), .b(new_n224), .c(new_n221), .d(new_n217), .o1(\s[20] ));
  nano23aa1n06x5               g131(.a(new_n214), .b(new_n222), .c(new_n223), .d(new_n215), .out0(new_n227));
  nand23aa1n06x5               g132(.a(new_n227), .b(new_n205), .c(new_n207), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  oaoi03aa1n02x5               g134(.a(\a[20] ), .b(\b[19] ), .c(new_n221), .o1(new_n230));
  tech160nm_fiao0012aa1n02p5x5 g135(.a(new_n230), .b(new_n227), .c(new_n213), .o(new_n231));
  xorc02aa1n02x5               g136(.a(\a[21] ), .b(\b[20] ), .out0(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n231), .c(new_n188), .d(new_n229), .o1(new_n233));
  aoi112aa1n02x5               g138(.a(new_n232), .b(new_n231), .c(new_n188), .d(new_n229), .o1(new_n234));
  norb02aa1n02x5               g139(.a(new_n233), .b(new_n234), .out0(\s[21] ));
  nor042aa1n03x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  xorc02aa1n02x5               g142(.a(\a[22] ), .b(\b[21] ), .out0(new_n238));
  inv000aa1d42x5               g143(.a(\a[22] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(\b[21] ), .o1(new_n240));
  aoi012aa1n02x5               g145(.a(new_n236), .b(new_n239), .c(new_n240), .o1(new_n241));
  oai112aa1n03x5               g146(.a(new_n233), .b(new_n241), .c(new_n240), .d(new_n239), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n238), .c(new_n237), .d(new_n233), .o1(\s[22] ));
  inv000aa1d42x5               g148(.a(\a[21] ), .o1(new_n244));
  xroi22aa1d04x5               g149(.a(new_n244), .b(\b[20] ), .c(new_n239), .d(\b[21] ), .out0(new_n245));
  norb02aa1n02x5               g150(.a(new_n245), .b(new_n228), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n245), .b(new_n230), .c(new_n227), .d(new_n213), .o1(new_n247));
  oaoi03aa1n09x5               g152(.a(new_n239), .b(new_n240), .c(new_n236), .o1(new_n248));
  nanp02aa1n02x5               g153(.a(new_n247), .b(new_n248), .o1(new_n249));
  xorc02aa1n02x5               g154(.a(\a[23] ), .b(\b[22] ), .out0(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n249), .c(new_n188), .d(new_n246), .o1(new_n251));
  aoi112aa1n02x5               g156(.a(new_n250), .b(new_n249), .c(new_n188), .d(new_n246), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n251), .b(new_n252), .out0(\s[23] ));
  nor042aa1n02x5               g158(.a(\b[22] ), .b(\a[23] ), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  xorc02aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .out0(new_n256));
  inv000aa1d42x5               g161(.a(\a[24] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(\b[23] ), .o1(new_n258));
  aoi012aa1n02x5               g163(.a(new_n254), .b(new_n257), .c(new_n258), .o1(new_n259));
  oai112aa1n03x5               g164(.a(new_n251), .b(new_n259), .c(new_n258), .d(new_n257), .o1(new_n260));
  aoai13aa1n03x5               g165(.a(new_n260), .b(new_n256), .c(new_n255), .d(new_n251), .o1(\s[24] ));
  inv000aa1d42x5               g166(.a(\a[23] ), .o1(new_n262));
  xroi22aa1d04x5               g167(.a(new_n262), .b(\b[22] ), .c(new_n257), .d(\b[23] ), .out0(new_n263));
  nano22aa1n02x5               g168(.a(new_n228), .b(new_n245), .c(new_n263), .out0(new_n264));
  inv000aa1n02x5               g169(.a(new_n263), .o1(new_n265));
  oaoi03aa1n02x5               g170(.a(new_n257), .b(new_n258), .c(new_n254), .o1(new_n266));
  aoai13aa1n04x5               g171(.a(new_n266), .b(new_n265), .c(new_n247), .d(new_n248), .o1(new_n267));
  xorc02aa1n02x5               g172(.a(\a[25] ), .b(\b[24] ), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n267), .c(new_n188), .d(new_n264), .o1(new_n269));
  aoi112aa1n02x5               g174(.a(new_n268), .b(new_n267), .c(new_n188), .d(new_n264), .o1(new_n270));
  norb02aa1n02x5               g175(.a(new_n269), .b(new_n270), .out0(\s[25] ));
  nor042aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .out0(new_n274));
  inv000aa1d42x5               g179(.a(\a[26] ), .o1(new_n275));
  inv000aa1d42x5               g180(.a(\b[25] ), .o1(new_n276));
  aoi012aa1n02x5               g181(.a(new_n272), .b(new_n275), .c(new_n276), .o1(new_n277));
  oai112aa1n03x5               g182(.a(new_n269), .b(new_n277), .c(new_n276), .d(new_n275), .o1(new_n278));
  aoai13aa1n03x5               g183(.a(new_n278), .b(new_n274), .c(new_n273), .d(new_n269), .o1(\s[26] ));
  and002aa1n02x5               g184(.a(new_n274), .b(new_n268), .o(new_n280));
  nanp02aa1n03x5               g185(.a(new_n267), .b(new_n280), .o1(new_n281));
  nano32aa1n03x7               g186(.a(new_n228), .b(new_n280), .c(new_n245), .d(new_n263), .out0(new_n282));
  aoai13aa1n06x5               g187(.a(new_n282), .b(new_n204), .c(new_n197), .d(new_n200), .o1(new_n283));
  oaoi03aa1n02x5               g188(.a(new_n275), .b(new_n276), .c(new_n272), .o1(new_n284));
  nand23aa1n03x5               g189(.a(new_n281), .b(new_n283), .c(new_n284), .o1(new_n285));
  xorb03aa1n03x5               g190(.a(new_n285), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n287), .o1(new_n288));
  xorc02aa1n12x5               g193(.a(\a[27] ), .b(\b[26] ), .out0(new_n289));
  nanp02aa1n02x5               g194(.a(new_n285), .b(new_n289), .o1(new_n290));
  xorc02aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .out0(new_n291));
  aobi12aa1n06x5               g196(.a(new_n284), .b(new_n188), .c(new_n282), .out0(new_n292));
  inv000aa1d42x5               g197(.a(new_n289), .o1(new_n293));
  oai022aa1n02x5               g198(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(\a[28] ), .c(\b[27] ), .o1(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n293), .c(new_n292), .d(new_n281), .o1(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n291), .c(new_n290), .d(new_n288), .o1(\s[28] ));
  and002aa1n02x5               g202(.a(new_n291), .b(new_n289), .o(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  inv000aa1d42x5               g204(.a(\b[27] ), .o1(new_n300));
  oaib12aa1n09x5               g205(.a(new_n294), .b(new_n300), .c(\a[28] ), .out0(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[28] ), .b(\a[29] ), .out0(new_n302));
  norb02aa1n02x5               g207(.a(new_n301), .b(new_n302), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n299), .c(new_n292), .d(new_n281), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n301), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n302), .b(new_n305), .c(new_n285), .d(new_n298), .o1(new_n306));
  nanp02aa1n03x5               g211(.a(new_n306), .b(new_n304), .o1(\s[29] ));
  nanp02aa1n02x5               g212(.a(\b[0] ), .b(\a[1] ), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g214(.a(new_n302), .b(new_n289), .c(new_n291), .out0(new_n310));
  oaoi03aa1n02x5               g215(.a(\a[29] ), .b(\b[28] ), .c(new_n301), .o1(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[29] ), .b(\a[30] ), .out0(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n311), .c(new_n285), .d(new_n310), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n310), .o1(new_n314));
  norp02aa1n02x5               g219(.a(new_n311), .b(new_n312), .o1(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n314), .c(new_n292), .d(new_n281), .o1(new_n316));
  nanp02aa1n03x5               g221(.a(new_n313), .b(new_n316), .o1(\s[30] ));
  nano23aa1n02x4               g222(.a(new_n312), .b(new_n302), .c(new_n291), .d(new_n289), .out0(new_n318));
  nanp02aa1n02x5               g223(.a(new_n285), .b(new_n318), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[31] ), .b(\b[30] ), .out0(new_n320));
  inv000aa1n02x5               g225(.a(new_n318), .o1(new_n321));
  oai012aa1n02x5               g226(.a(new_n320), .b(\b[29] ), .c(\a[30] ), .o1(new_n322));
  aoib12aa1n02x5               g227(.a(new_n322), .b(new_n311), .c(new_n312), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n321), .c(new_n292), .d(new_n281), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .c(new_n315), .carry(new_n325));
  aoai13aa1n03x5               g230(.a(new_n324), .b(new_n320), .c(new_n319), .d(new_n325), .o1(\s[31] ));
  norb02aa1n02x5               g231(.a(new_n98), .b(new_n99), .out0(new_n327));
  xobna2aa1n03x5               g232(.a(new_n327), .b(new_n101), .c(new_n97), .out0(\s[3] ));
  inv000aa1d42x5               g233(.a(new_n99), .o1(new_n329));
  nanp03aa1n02x5               g234(.a(new_n327), .b(new_n97), .c(new_n101), .o1(new_n330));
  xnbna2aa1n03x5               g235(.a(new_n193), .b(new_n330), .c(new_n329), .out0(\s[4] ));
  xorb03aa1n02x5               g236(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g237(.a(new_n107), .b(new_n113), .o1(new_n333));
  xnbna2aa1n03x5               g238(.a(new_n110), .b(new_n333), .c(new_n121), .out0(\s[6] ));
  inv000aa1d42x5               g239(.a(new_n116), .o1(new_n335));
  aoi012aa1n02x5               g240(.a(new_n122), .b(new_n107), .c(new_n195), .o1(new_n336));
  xnbna2aa1n03x5               g241(.a(new_n336), .b(new_n335), .c(new_n117), .out0(\s[7] ));
  nanb02aa1n02x5               g242(.a(new_n114), .b(new_n115), .out0(new_n338));
  nanb03aa1n02x5               g243(.a(new_n336), .b(new_n335), .c(new_n117), .out0(new_n339));
  xobna2aa1n03x5               g244(.a(new_n338), .b(new_n339), .c(new_n335), .out0(\s[8] ));
  xorb03aa1n02x5               g245(.a(new_n197), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


