// Benchmark "adder" written by ABC on Wed Jul 17 20:57:25 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n332, new_n334, new_n336, new_n337, new_n338, new_n340,
    new_n342, new_n343, new_n344, new_n345, new_n346, new_n348;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nand42aa1n04x5               g002(.a(\b[2] ), .b(\a[3] ), .o1(new_n98));
  nand42aa1n20x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nor022aa1n06x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nano22aa1n06x5               g005(.a(new_n100), .b(new_n98), .c(new_n99), .out0(new_n101));
  nand22aa1n09x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  norp02aa1n04x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nona22aa1n09x5               g008(.a(new_n99), .b(new_n103), .c(new_n102), .out0(new_n104));
  nand42aa1n08x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  orn002aa1n24x5               g010(.a(\a[4] ), .b(\b[3] ), .o(new_n106));
  oai112aa1n06x5               g011(.a(new_n106), .b(new_n105), .c(\b[2] ), .d(\a[3] ), .o1(new_n107));
  aoi012aa1n12x5               g012(.a(new_n107), .b(new_n101), .c(new_n104), .o1(new_n108));
  nand02aa1n16x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nanp02aa1n03x5               g014(.a(new_n109), .b(new_n105), .o1(new_n110));
  nor042aa1d18x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand22aa1n04x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  norb02aa1n03x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nand02aa1n08x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  oai012aa1n03x5               g019(.a(new_n114), .b(\b[5] ), .c(\a[6] ), .o1(new_n115));
  nor002aa1d32x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nor042aa1n04x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  tech160nm_finand02aa1n05x5   g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  norb03aa1n12x5               g023(.a(new_n118), .b(new_n116), .c(new_n117), .out0(new_n119));
  nona23aa1n09x5               g024(.a(new_n119), .b(new_n113), .c(new_n110), .d(new_n115), .out0(new_n120));
  inv030aa1n04x5               g025(.a(new_n111), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[8] ), .b(\b[7] ), .c(new_n121), .o1(new_n122));
  nanb03aa1n03x5               g027(.a(new_n117), .b(new_n118), .c(new_n112), .out0(new_n123));
  nor042aa1n04x5               g028(.a(\b[5] ), .b(\a[6] ), .o1(new_n124));
  oai112aa1n02x7               g029(.a(new_n121), .b(new_n109), .c(new_n124), .d(new_n116), .o1(new_n125));
  oab012aa1n09x5               g030(.a(new_n122), .b(new_n125), .c(new_n123), .out0(new_n126));
  oai012aa1n12x5               g031(.a(new_n126), .b(new_n108), .c(new_n120), .o1(new_n127));
  nanp02aa1n04x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  aoi012aa1n02x5               g033(.a(new_n97), .b(new_n127), .c(new_n128), .o1(new_n129));
  nor002aa1n04x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand02aa1n06x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  norb02aa1n02x5               g037(.a(new_n128), .b(new_n97), .out0(new_n133));
  oai022aa1d24x5               g038(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n134));
  aoi122aa1n02x5               g039(.a(new_n134), .b(\b[9] ), .c(\a[10] ), .d(new_n127), .e(new_n133), .o1(new_n135));
  oabi12aa1n02x5               g040(.a(new_n135), .b(new_n129), .c(new_n132), .out0(\s[10] ));
  nano23aa1n06x5               g041(.a(new_n97), .b(new_n130), .c(new_n131), .d(new_n128), .out0(new_n137));
  oa0012aa1n02x5               g042(.a(new_n131), .b(new_n130), .c(new_n97), .o(new_n138));
  xorc02aa1n02x5               g043(.a(\a[11] ), .b(\b[10] ), .out0(new_n139));
  aoai13aa1n02x5               g044(.a(new_n139), .b(new_n138), .c(new_n127), .d(new_n137), .o1(new_n140));
  aoi112aa1n02x5               g045(.a(new_n139), .b(new_n138), .c(new_n127), .d(new_n137), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(\s[11] ));
  orn002aa1n24x5               g047(.a(\a[11] ), .b(\b[10] ), .o(new_n143));
  nor042aa1n04x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand02aa1n03x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n06x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  oab012aa1n02x4               g051(.a(new_n144), .b(\a[11] ), .c(\b[10] ), .out0(new_n147));
  nanp03aa1n02x5               g052(.a(new_n140), .b(new_n145), .c(new_n147), .o1(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n146), .c(new_n143), .d(new_n140), .o1(\s[12] ));
  nona23aa1n02x4               g054(.a(new_n131), .b(new_n128), .c(new_n97), .d(new_n130), .out0(new_n150));
  nand42aa1n02x5               g055(.a(\b[10] ), .b(\a[11] ), .o1(new_n151));
  nano32aa1n03x7               g056(.a(new_n150), .b(new_n146), .c(new_n143), .d(new_n151), .out0(new_n152));
  nanb03aa1n06x5               g057(.a(new_n144), .b(new_n145), .c(new_n151), .out0(new_n153));
  oai112aa1n06x5               g058(.a(new_n134), .b(new_n131), .c(\b[10] ), .d(\a[11] ), .o1(new_n154));
  oaoi03aa1n03x5               g059(.a(\a[12] ), .b(\b[11] ), .c(new_n143), .o1(new_n155));
  oabi12aa1n06x5               g060(.a(new_n155), .b(new_n153), .c(new_n154), .out0(new_n156));
  nor042aa1n06x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand42aa1n16x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n156), .c(new_n127), .d(new_n152), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(new_n159), .b(new_n156), .c(new_n127), .d(new_n152), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n160), .b(new_n161), .out0(\s[13] ));
  inv000aa1n02x5               g067(.a(new_n157), .o1(new_n163));
  nor042aa1n04x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand42aa1n10x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  nona23aa1n02x4               g071(.a(new_n160), .b(new_n165), .c(new_n164), .d(new_n157), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n166), .c(new_n163), .d(new_n160), .o1(\s[14] ));
  nano23aa1d15x5               g073(.a(new_n157), .b(new_n164), .c(new_n165), .d(new_n158), .out0(new_n169));
  oaoi03aa1n02x5               g074(.a(\a[14] ), .b(\b[13] ), .c(new_n163), .o1(new_n170));
  aoi012aa1n02x5               g075(.a(new_n170), .b(new_n156), .c(new_n169), .o1(new_n171));
  nand03aa1n02x5               g076(.a(new_n137), .b(new_n139), .c(new_n146), .o1(new_n172));
  inv000aa1d42x5               g077(.a(new_n169), .o1(new_n173));
  nona22aa1n02x4               g078(.a(new_n127), .b(new_n172), .c(new_n173), .out0(new_n174));
  nor042aa1n02x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nand42aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  xnbna2aa1n03x5               g082(.a(new_n177), .b(new_n174), .c(new_n171), .out0(\s[15] ));
  nanp02aa1n02x5               g083(.a(new_n174), .b(new_n171), .o1(new_n179));
  aoi012aa1n02x5               g084(.a(new_n175), .b(new_n179), .c(new_n177), .o1(new_n180));
  nor042aa1n02x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nanp02aa1n04x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n182), .b(new_n181), .out0(new_n183));
  oai022aa1n02x5               g088(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n184));
  aoi122aa1n02x5               g089(.a(new_n184), .b(\b[15] ), .c(\a[16] ), .d(new_n179), .e(new_n177), .o1(new_n185));
  oabi12aa1n02x5               g090(.a(new_n185), .b(new_n180), .c(new_n183), .out0(\s[16] ));
  nano23aa1n06x5               g091(.a(new_n175), .b(new_n181), .c(new_n182), .d(new_n176), .out0(new_n187));
  aoai13aa1n12x5               g092(.a(new_n187), .b(new_n170), .c(new_n156), .d(new_n169), .o1(new_n188));
  nano22aa1n03x7               g093(.a(new_n172), .b(new_n169), .c(new_n187), .out0(new_n189));
  nanp02aa1n04x5               g094(.a(new_n127), .b(new_n189), .o1(new_n190));
  oai012aa1n02x5               g095(.a(new_n182), .b(new_n181), .c(new_n175), .o1(new_n191));
  nanp03aa1d12x5               g096(.a(new_n190), .b(new_n188), .c(new_n191), .o1(new_n192));
  xorc02aa1n06x5               g097(.a(\a[17] ), .b(\b[16] ), .out0(new_n193));
  aoi122aa1n02x5               g098(.a(new_n193), .b(new_n182), .c(new_n184), .d(new_n127), .e(new_n189), .o1(new_n194));
  aoi022aa1n02x5               g099(.a(new_n194), .b(new_n188), .c(new_n192), .d(new_n193), .o1(\s[17] ));
  nor042aa1n03x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  inv040aa1n08x5               g102(.a(new_n188), .o1(new_n198));
  nanb03aa1n02x5               g103(.a(new_n100), .b(new_n98), .c(new_n99), .out0(new_n199));
  norb03aa1n02x5               g104(.a(new_n99), .b(new_n103), .c(new_n102), .out0(new_n200));
  oabi12aa1n06x5               g105(.a(new_n107), .b(new_n200), .c(new_n199), .out0(new_n201));
  aoi022aa1n02x5               g106(.a(\b[5] ), .b(\a[6] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n202));
  nona23aa1n02x4               g107(.a(new_n114), .b(new_n112), .c(new_n124), .d(new_n111), .out0(new_n203));
  nano22aa1n03x7               g108(.a(new_n203), .b(new_n119), .c(new_n202), .out0(new_n204));
  nand42aa1n03x5               g109(.a(new_n204), .b(new_n201), .o1(new_n205));
  nanp03aa1n02x5               g110(.a(new_n152), .b(new_n169), .c(new_n187), .o1(new_n206));
  aoai13aa1n06x5               g111(.a(new_n191), .b(new_n206), .c(new_n205), .d(new_n126), .o1(new_n207));
  oai012aa1n02x5               g112(.a(new_n193), .b(new_n207), .c(new_n198), .o1(new_n208));
  tech160nm_fixorc02aa1n02p5x5 g113(.a(\a[18] ), .b(\b[17] ), .out0(new_n209));
  oai022aa1d18x5               g114(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n210));
  aoi012aa1n02x5               g115(.a(new_n210), .b(\a[18] ), .c(\b[17] ), .o1(new_n211));
  nanp02aa1n02x5               g116(.a(new_n208), .b(new_n211), .o1(new_n212));
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n209), .c(new_n197), .d(new_n208), .o1(\s[18] ));
  and002aa1n06x5               g118(.a(new_n209), .b(new_n193), .o(new_n214));
  oaoi03aa1n02x5               g119(.a(\a[18] ), .b(\b[17] ), .c(new_n197), .o1(new_n215));
  tech160nm_fixorc02aa1n03p5x5 g120(.a(\a[19] ), .b(\b[18] ), .out0(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n215), .c(new_n192), .d(new_n214), .o1(new_n217));
  aoi112aa1n02x7               g122(.a(new_n216), .b(new_n215), .c(new_n192), .d(new_n214), .o1(new_n218));
  norb02aa1n03x4               g123(.a(new_n217), .b(new_n218), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g125(.a(\a[19] ), .o1(new_n221));
  nanb02aa1n02x5               g126(.a(\b[18] ), .b(new_n221), .out0(new_n222));
  xorc02aa1n02x5               g127(.a(\a[20] ), .b(\b[19] ), .out0(new_n223));
  nand42aa1d28x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  oai022aa1d18x5               g129(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n224), .b(new_n225), .out0(new_n226));
  nanp02aa1n03x5               g131(.a(new_n217), .b(new_n226), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n223), .c(new_n217), .d(new_n222), .o1(\s[20] ));
  and002aa1n06x5               g133(.a(new_n223), .b(new_n216), .o(new_n229));
  and002aa1n02x5               g134(.a(new_n229), .b(new_n214), .o(new_n230));
  aoi022aa1d24x5               g135(.a(\b[18] ), .b(\a[19] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n231));
  aoai13aa1n12x5               g136(.a(new_n224), .b(new_n225), .c(new_n210), .d(new_n231), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  nor042aa1n12x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  nand02aa1n03x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n233), .c(new_n192), .d(new_n230), .o1(new_n237));
  aoi112aa1n02x7               g142(.a(new_n236), .b(new_n233), .c(new_n192), .d(new_n230), .o1(new_n238));
  norb02aa1n03x4               g143(.a(new_n237), .b(new_n238), .out0(\s[21] ));
  inv000aa1d42x5               g144(.a(new_n234), .o1(new_n240));
  nor042aa1n04x5               g145(.a(\b[21] ), .b(\a[22] ), .o1(new_n241));
  nanp02aa1n04x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(new_n243));
  norb03aa1n02x5               g148(.a(new_n242), .b(new_n234), .c(new_n241), .out0(new_n244));
  nand42aa1n02x5               g149(.a(new_n237), .b(new_n244), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n245), .b(new_n243), .c(new_n237), .d(new_n240), .o1(\s[22] ));
  nona23aa1d18x5               g151(.a(new_n242), .b(new_n235), .c(new_n234), .d(new_n241), .out0(new_n247));
  inv040aa1n06x5               g152(.a(new_n247), .o1(new_n248));
  nanp03aa1d12x5               g153(.a(new_n229), .b(new_n214), .c(new_n248), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  oaih12aa1n02x5               g155(.a(new_n242), .b(new_n241), .c(new_n234), .o1(new_n251));
  oa0012aa1n02x5               g156(.a(new_n251), .b(new_n232), .c(new_n247), .o(new_n252));
  inv000aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  xorc02aa1n12x5               g158(.a(\a[23] ), .b(\b[22] ), .out0(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n253), .c(new_n192), .d(new_n250), .o1(new_n255));
  aoi112aa1n02x7               g160(.a(new_n254), .b(new_n253), .c(new_n192), .d(new_n250), .o1(new_n256));
  norb02aa1n03x4               g161(.a(new_n255), .b(new_n256), .out0(\s[23] ));
  nor042aa1n03x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  tech160nm_fixorc02aa1n02p5x5 g164(.a(\a[24] ), .b(\b[23] ), .out0(new_n260));
  nanp02aa1n02x5               g165(.a(\b[23] ), .b(\a[24] ), .o1(new_n261));
  oai022aa1n02x5               g166(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n261), .b(new_n262), .out0(new_n263));
  nanp02aa1n03x5               g168(.a(new_n255), .b(new_n263), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n260), .c(new_n255), .d(new_n259), .o1(\s[24] ));
  nand02aa1n06x5               g170(.a(new_n260), .b(new_n254), .o1(new_n266));
  nano32aa1n02x4               g171(.a(new_n266), .b(new_n229), .c(new_n214), .d(new_n248), .out0(new_n267));
  oai012aa1n02x7               g172(.a(new_n267), .b(new_n207), .c(new_n198), .o1(new_n268));
  oaoi13aa1n09x5               g173(.a(new_n266), .b(new_n251), .c(new_n232), .d(new_n247), .o1(new_n269));
  tech160nm_fioaoi03aa1n03p5x5 g174(.a(\a[24] ), .b(\b[23] ), .c(new_n259), .o1(new_n270));
  norp02aa1n02x5               g175(.a(new_n269), .b(new_n270), .o1(new_n271));
  tech160nm_fixorc02aa1n03p5x5 g176(.a(\a[25] ), .b(\b[24] ), .out0(new_n272));
  xnbna2aa1n03x5               g177(.a(new_n272), .b(new_n268), .c(new_n271), .out0(\s[25] ));
  nor042aa1n03x5               g178(.a(\b[24] ), .b(\a[25] ), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  inv000aa1n02x5               g180(.a(new_n271), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n272), .b(new_n276), .c(new_n192), .d(new_n267), .o1(new_n277));
  xorc02aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .out0(new_n278));
  oai022aa1n02x5               g183(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n279));
  aoi012aa1n02x5               g184(.a(new_n279), .b(\a[26] ), .c(\b[25] ), .o1(new_n280));
  nand42aa1n02x5               g185(.a(new_n277), .b(new_n280), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n278), .c(new_n277), .d(new_n275), .o1(\s[26] ));
  inv000aa1d42x5               g187(.a(new_n266), .o1(new_n283));
  and002aa1n02x5               g188(.a(new_n278), .b(new_n272), .o(new_n284));
  nano22aa1n06x5               g189(.a(new_n249), .b(new_n283), .c(new_n284), .out0(new_n285));
  tech160nm_fioai012aa1n04x5   g190(.a(new_n285), .b(new_n207), .c(new_n198), .o1(new_n286));
  aoi022aa1n02x5               g191(.a(new_n127), .b(new_n189), .c(new_n182), .d(new_n184), .o1(new_n287));
  inv000aa1n02x5               g192(.a(new_n285), .o1(new_n288));
  oaoi03aa1n02x5               g193(.a(\a[26] ), .b(\b[25] ), .c(new_n275), .o1(new_n289));
  oaoi13aa1n04x5               g194(.a(new_n289), .b(new_n284), .c(new_n269), .d(new_n270), .o1(new_n290));
  aoai13aa1n06x5               g195(.a(new_n290), .b(new_n288), .c(new_n287), .d(new_n188), .o1(new_n291));
  xorc02aa1n12x5               g196(.a(\a[27] ), .b(\b[26] ), .out0(new_n292));
  aoi112aa1n02x5               g197(.a(new_n292), .b(new_n289), .c(new_n276), .d(new_n284), .o1(new_n293));
  aoi022aa1n02x5               g198(.a(new_n293), .b(new_n286), .c(new_n291), .d(new_n292), .o1(\s[27] ));
  norp02aa1n02x5               g199(.a(\b[26] ), .b(\a[27] ), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  nanp02aa1n03x5               g201(.a(new_n291), .b(new_n292), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .out0(new_n298));
  inv000aa1d42x5               g203(.a(new_n292), .o1(new_n299));
  oai022aa1n02x5               g204(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(\a[28] ), .c(\b[27] ), .o1(new_n301));
  aoai13aa1n02x5               g206(.a(new_n301), .b(new_n299), .c(new_n286), .d(new_n290), .o1(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n298), .c(new_n297), .d(new_n296), .o1(\s[28] ));
  and002aa1n12x5               g208(.a(new_n298), .b(new_n292), .o(new_n304));
  inv000aa1d42x5               g209(.a(new_n304), .o1(new_n305));
  inv000aa1d42x5               g210(.a(\b[27] ), .o1(new_n306));
  oaib12aa1n09x5               g211(.a(new_n300), .b(new_n306), .c(\a[28] ), .out0(new_n307));
  inv000aa1d42x5               g212(.a(new_n307), .o1(new_n308));
  tech160nm_fixorc02aa1n03p5x5 g213(.a(\a[29] ), .b(\b[28] ), .out0(new_n309));
  norb02aa1n02x5               g214(.a(new_n309), .b(new_n308), .out0(new_n310));
  aoai13aa1n02x5               g215(.a(new_n310), .b(new_n305), .c(new_n286), .d(new_n290), .o1(new_n311));
  inv000aa1d42x5               g216(.a(new_n309), .o1(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n308), .c(new_n291), .d(new_n304), .o1(new_n313));
  nanp02aa1n03x5               g218(.a(new_n313), .b(new_n311), .o1(\s[29] ));
  xorb03aa1n02x5               g219(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g220(.a(new_n312), .b(new_n292), .c(new_n298), .out0(new_n316));
  oaoi03aa1n02x5               g221(.a(\a[29] ), .b(\b[28] ), .c(new_n307), .o1(new_n317));
  xnrc02aa1n02x5               g222(.a(\b[29] ), .b(\a[30] ), .out0(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n317), .c(new_n291), .d(new_n316), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n316), .o1(new_n320));
  norp02aa1n02x5               g225(.a(new_n317), .b(new_n318), .o1(new_n321));
  aoai13aa1n02x5               g226(.a(new_n321), .b(new_n320), .c(new_n286), .d(new_n290), .o1(new_n322));
  nanp02aa1n03x5               g227(.a(new_n319), .b(new_n322), .o1(\s[30] ));
  nano32aa1n03x7               g228(.a(new_n318), .b(new_n309), .c(new_n298), .d(new_n292), .out0(new_n324));
  aoi012aa1n02x5               g229(.a(new_n321), .b(\a[30] ), .c(\b[29] ), .o1(new_n325));
  xnrc02aa1n02x5               g230(.a(\b[30] ), .b(\a[31] ), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n325), .c(new_n291), .d(new_n324), .o1(new_n327));
  inv000aa1n02x5               g232(.a(new_n324), .o1(new_n328));
  norp02aa1n02x5               g233(.a(new_n325), .b(new_n326), .o1(new_n329));
  aoai13aa1n02x5               g234(.a(new_n329), .b(new_n328), .c(new_n286), .d(new_n290), .o1(new_n330));
  nanp02aa1n03x5               g235(.a(new_n327), .b(new_n330), .o1(\s[31] ));
  aboi22aa1n03x5               g236(.a(new_n100), .b(new_n98), .c(new_n104), .d(new_n99), .out0(new_n332));
  aoi012aa1n02x5               g237(.a(new_n332), .b(new_n101), .c(new_n104), .o1(\s[3] ));
  aoi012aa1n02x5               g238(.a(new_n100), .b(new_n101), .c(new_n104), .o1(new_n334));
  aoai13aa1n02x5               g239(.a(new_n201), .b(new_n334), .c(new_n106), .d(new_n105), .o1(\s[4] ));
  nona23aa1n09x5               g240(.a(new_n105), .b(new_n114), .c(new_n108), .d(new_n116), .out0(new_n336));
  inv000aa1d42x5               g241(.a(new_n116), .o1(new_n337));
  aoi022aa1n02x5               g242(.a(new_n201), .b(new_n105), .c(new_n337), .d(new_n114), .o1(new_n338));
  norb02aa1n02x5               g243(.a(new_n336), .b(new_n338), .out0(\s[5] ));
  norb02aa1n02x5               g244(.a(new_n109), .b(new_n124), .out0(new_n340));
  xnbna2aa1n03x5               g245(.a(new_n340), .b(new_n336), .c(new_n337), .out0(\s[6] ));
  inv000aa1d42x5               g246(.a(new_n109), .o1(new_n342));
  nona32aa1n02x4               g247(.a(new_n336), .b(new_n116), .c(new_n124), .d(new_n342), .out0(new_n343));
  nano22aa1n02x4               g248(.a(new_n111), .b(new_n109), .c(new_n112), .out0(new_n344));
  nanp02aa1n03x5               g249(.a(new_n343), .b(new_n344), .o1(new_n345));
  aoi022aa1n02x5               g250(.a(new_n343), .b(new_n109), .c(new_n121), .d(new_n112), .o1(new_n346));
  norb02aa1n02x5               g251(.a(new_n345), .b(new_n346), .out0(\s[7] ));
  norb02aa1n02x5               g252(.a(new_n118), .b(new_n117), .out0(new_n348));
  xnbna2aa1n03x5               g253(.a(new_n348), .b(new_n345), .c(new_n121), .out0(\s[8] ));
  xorb03aa1n02x5               g254(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


