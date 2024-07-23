// Benchmark "adder" written by ABC on Wed Jul 17 19:18:21 2024

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
    new_n155, new_n156, new_n157, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n300, new_n301, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n352,
    new_n355, new_n356, new_n357, new_n359, new_n360, new_n362, new_n363;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1n06x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nanp02aa1n06x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nanb02aa1n06x5               g005(.a(new_n99), .b(new_n100), .out0(new_n101));
  nand42aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  oai012aa1n03x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nand42aa1n10x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nor002aa1n02x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  norb03aa1n03x5               g012(.a(new_n106), .b(new_n99), .c(new_n107), .out0(new_n108));
  oai012aa1n09x5               g013(.a(new_n108), .b(new_n105), .c(new_n101), .o1(new_n109));
  nand02aa1n06x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  aoi022aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n111));
  oai112aa1n02x5               g016(.a(new_n111), .b(new_n110), .c(\b[7] ), .d(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanb02aa1n09x5               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  oai122aa1n02x7               g020(.a(new_n106), .b(\a[7] ), .c(\b[6] ), .d(\a[6] ), .e(\b[5] ), .o1(new_n116));
  nor043aa1n03x5               g021(.a(new_n112), .b(new_n115), .c(new_n116), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nand23aa1n03x5               g024(.a(new_n118), .b(new_n110), .c(new_n119), .o1(new_n120));
  oai022aa1n03x5               g025(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n121), .b(new_n118), .o1(new_n122));
  nor042aa1n02x5               g027(.a(\b[5] ), .b(\a[6] ), .o1(new_n123));
  norb03aa1n03x5               g028(.a(new_n110), .b(new_n123), .c(new_n114), .out0(new_n124));
  oai013aa1n03x5               g029(.a(new_n122), .b(new_n124), .c(new_n120), .d(new_n121), .o1(new_n125));
  nand42aa1n03x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n97), .out0(new_n127));
  aoai13aa1n06x5               g032(.a(new_n127), .b(new_n125), .c(new_n117), .d(new_n109), .o1(new_n128));
  nor042aa1n06x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand42aa1n04x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  oai112aa1n06x5               g037(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n133));
  nanb02aa1n02x5               g038(.a(new_n107), .b(new_n106), .out0(new_n134));
  aoi113aa1n03x7               g039(.a(new_n134), .b(new_n99), .c(new_n133), .d(new_n100), .e(new_n102), .o1(new_n135));
  norp02aa1n02x5               g040(.a(\b[7] ), .b(\a[8] ), .o1(new_n136));
  nano32aa1n02x4               g041(.a(new_n136), .b(new_n119), .c(new_n110), .d(new_n118), .out0(new_n137));
  nona22aa1n02x4               g042(.a(new_n137), .b(new_n115), .c(new_n116), .out0(new_n138));
  nor003aa1n02x5               g043(.a(new_n124), .b(new_n120), .c(new_n121), .o1(new_n139));
  norb02aa1n03x5               g044(.a(new_n122), .b(new_n139), .out0(new_n140));
  oaih12aa1n06x5               g045(.a(new_n140), .b(new_n138), .c(new_n135), .o1(new_n141));
  aoai13aa1n02x5               g046(.a(new_n131), .b(new_n97), .c(new_n141), .d(new_n126), .o1(new_n142));
  oai012aa1n18x5               g047(.a(new_n130), .b(new_n129), .c(new_n97), .o1(new_n143));
  norp02aa1n24x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  nand22aa1n03x5               g049(.a(\b[10] ), .b(\a[11] ), .o1(new_n145));
  norb02aa1n06x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n146), .b(new_n142), .c(new_n143), .out0(\s[11] ));
  inv000aa1d42x5               g052(.a(new_n144), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n128), .b(new_n98), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n143), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n146), .b(new_n150), .c(new_n149), .d(new_n131), .o1(new_n151));
  nor002aa1n04x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nand42aa1n06x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  norb02aa1n09x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  inv040aa1n03x5               g059(.a(new_n154), .o1(new_n155));
  tech160nm_fiaoi012aa1n03p5x5 g060(.a(new_n155), .b(new_n151), .c(new_n148), .o1(new_n156));
  nona22aa1n02x5               g061(.a(new_n151), .b(new_n154), .c(new_n144), .out0(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(\s[12] ));
  norb03aa1n02x7               g063(.a(new_n130), .b(new_n97), .c(new_n129), .out0(new_n159));
  nano32aa1n03x7               g064(.a(new_n155), .b(new_n159), .c(new_n146), .d(new_n126), .out0(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n125), .c(new_n117), .d(new_n109), .o1(new_n161));
  nona23aa1n09x5               g066(.a(new_n153), .b(new_n145), .c(new_n144), .d(new_n152), .out0(new_n162));
  tech160nm_fiaoi012aa1n05x5   g067(.a(new_n152), .b(new_n144), .c(new_n153), .o1(new_n163));
  oai012aa1n12x5               g068(.a(new_n163), .b(new_n162), .c(new_n143), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  nor042aa1n06x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  nanp02aa1n09x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n168), .b(new_n161), .c(new_n165), .out0(\s[13] ));
  aoi012aa1n03x5               g074(.a(new_n164), .b(new_n141), .c(new_n160), .o1(new_n170));
  oaoi03aa1n02x5               g075(.a(\a[13] ), .b(\b[12] ), .c(new_n170), .o1(new_n171));
  xorb03aa1n02x5               g076(.a(new_n171), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n06x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nand42aa1n08x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  tech160nm_fioai012aa1n03p5x5 g079(.a(new_n174), .b(new_n173), .c(new_n166), .o1(new_n175));
  nano23aa1d12x5               g080(.a(new_n166), .b(new_n173), .c(new_n174), .d(new_n167), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  aoai13aa1n06x5               g082(.a(new_n175), .b(new_n177), .c(new_n161), .d(new_n165), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nand22aa1n04x5               g085(.a(\b[14] ), .b(\a[15] ), .o1(new_n181));
  nor042aa1n03x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nanp02aa1n04x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  norb02aa1n02x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  aoai13aa1n03x5               g089(.a(new_n184), .b(new_n180), .c(new_n178), .d(new_n181), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n180), .b(new_n184), .c(new_n178), .d(new_n181), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n185), .b(new_n186), .out0(\s[16] ));
  aoi012aa1n06x5               g092(.a(new_n125), .b(new_n117), .c(new_n109), .o1(new_n188));
  nano32aa1n02x5               g093(.a(new_n155), .b(new_n148), .c(new_n145), .d(new_n126), .out0(new_n189));
  inv000aa1d42x5               g094(.a(\a[14] ), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(\b[13] ), .b(new_n190), .out0(new_n191));
  oai112aa1n02x5               g096(.a(new_n191), .b(new_n174), .c(\b[12] ), .d(\a[13] ), .o1(new_n192));
  inv000aa1n02x5               g097(.a(new_n180), .o1(new_n193));
  nanb03aa1n06x5               g098(.a(new_n182), .b(new_n183), .c(new_n181), .out0(new_n194));
  nano23aa1n03x7               g099(.a(new_n192), .b(new_n194), .c(new_n193), .d(new_n167), .out0(new_n195));
  nand23aa1n02x5               g100(.a(new_n195), .b(new_n159), .c(new_n189), .o1(new_n196));
  nona23aa1n02x4               g101(.a(new_n183), .b(new_n181), .c(new_n180), .d(new_n182), .out0(new_n197));
  oaoi03aa1n02x5               g102(.a(\a[16] ), .b(\b[15] ), .c(new_n193), .o1(new_n198));
  oabi12aa1n02x5               g103(.a(new_n198), .b(new_n197), .c(new_n175), .out0(new_n199));
  aoi012aa1n06x5               g104(.a(new_n199), .b(new_n164), .c(new_n195), .o1(new_n200));
  oai012aa1n12x5               g105(.a(new_n200), .b(new_n188), .c(new_n196), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g107(.a(\a[18] ), .o1(new_n203));
  inv040aa1d28x5               g108(.a(\a[17] ), .o1(new_n204));
  inv040aa1d32x5               g109(.a(\b[16] ), .o1(new_n205));
  oaoi03aa1n03x5               g110(.a(new_n204), .b(new_n205), .c(new_n201), .o1(new_n206));
  xorb03aa1n02x5               g111(.a(new_n206), .b(\b[17] ), .c(new_n203), .out0(\s[18] ));
  nona22aa1n03x5               g112(.a(new_n176), .b(new_n194), .c(new_n180), .out0(new_n208));
  nano22aa1n03x7               g113(.a(new_n208), .b(new_n189), .c(new_n159), .out0(new_n209));
  nanb03aa1n03x5               g114(.a(new_n143), .b(new_n154), .c(new_n146), .out0(new_n210));
  oab012aa1n03x5               g115(.a(new_n198), .b(new_n197), .c(new_n175), .out0(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n208), .c(new_n210), .d(new_n163), .o1(new_n212));
  xroi22aa1d06x4               g117(.a(new_n204), .b(\b[16] ), .c(new_n203), .d(\b[17] ), .out0(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n212), .c(new_n141), .d(new_n209), .o1(new_n214));
  nor022aa1n16x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  nand42aa1n16x5               g120(.a(\b[17] ), .b(\a[18] ), .o1(new_n216));
  aoai13aa1n12x5               g121(.a(new_n216), .b(new_n215), .c(new_n204), .d(new_n205), .o1(new_n217));
  xnrc02aa1n12x5               g122(.a(\b[18] ), .b(\a[19] ), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  xnbna2aa1n03x5               g124(.a(new_n219), .b(new_n214), .c(new_n217), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n217), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n219), .b(new_n224), .c(new_n201), .d(new_n213), .o1(new_n225));
  inv040aa1d32x5               g130(.a(\a[20] ), .o1(new_n226));
  inv040aa1d28x5               g131(.a(\b[19] ), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(new_n227), .b(new_n226), .o1(new_n228));
  nanp02aa1n02x5               g133(.a(\b[19] ), .b(\a[20] ), .o1(new_n229));
  nand02aa1d04x5               g134(.a(new_n228), .b(new_n229), .o1(new_n230));
  aoi012aa1n03x5               g135(.a(new_n230), .b(new_n225), .c(new_n223), .o1(new_n231));
  tech160nm_fiaoi012aa1n02p5x5 g136(.a(new_n218), .b(new_n214), .c(new_n217), .o1(new_n232));
  nano22aa1n02x4               g137(.a(new_n232), .b(new_n223), .c(new_n230), .out0(new_n233));
  norp02aa1n03x5               g138(.a(new_n231), .b(new_n233), .o1(\s[20] ));
  and002aa1n02x5               g139(.a(\b[18] ), .b(\a[19] ), .o(new_n235));
  nona32aa1n09x5               g140(.a(new_n213), .b(new_n230), .c(new_n235), .d(new_n222), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n212), .c(new_n141), .d(new_n209), .o1(new_n238));
  oaoi03aa1n12x5               g143(.a(new_n226), .b(new_n227), .c(new_n222), .o1(new_n239));
  oai013aa1d12x5               g144(.a(new_n239), .b(new_n218), .c(new_n217), .d(new_n230), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  nor042aa1n06x5               g146(.a(\b[20] ), .b(\a[21] ), .o1(new_n242));
  nand42aa1n04x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(new_n244));
  xnbna2aa1n03x5               g149(.a(new_n244), .b(new_n238), .c(new_n241), .out0(\s[21] ));
  inv000aa1n02x5               g150(.a(new_n242), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n244), .b(new_n240), .c(new_n201), .d(new_n237), .o1(new_n247));
  nor002aa1n16x5               g152(.a(\b[21] ), .b(\a[22] ), .o1(new_n248));
  nand42aa1n08x5               g153(.a(\b[21] ), .b(\a[22] ), .o1(new_n249));
  nanb02aa1n02x5               g154(.a(new_n248), .b(new_n249), .out0(new_n250));
  tech160nm_fiaoi012aa1n02p5x5 g155(.a(new_n250), .b(new_n247), .c(new_n246), .o1(new_n251));
  inv000aa1n02x5               g156(.a(new_n244), .o1(new_n252));
  tech160nm_fiaoi012aa1n04x5   g157(.a(new_n252), .b(new_n238), .c(new_n241), .o1(new_n253));
  nano22aa1n02x4               g158(.a(new_n253), .b(new_n246), .c(new_n250), .out0(new_n254));
  norp02aa1n03x5               g159(.a(new_n251), .b(new_n254), .o1(\s[22] ));
  nano23aa1n06x5               g160(.a(new_n242), .b(new_n248), .c(new_n249), .d(new_n243), .out0(new_n256));
  norb02aa1n09x5               g161(.a(new_n256), .b(new_n236), .out0(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n212), .c(new_n141), .d(new_n209), .o1(new_n258));
  inv040aa1n03x5               g163(.a(new_n248), .o1(new_n259));
  oai112aa1n06x5               g164(.a(new_n259), .b(new_n249), .c(\b[20] ), .d(\a[21] ), .o1(new_n260));
  aoi022aa1n09x5               g165(.a(new_n240), .b(new_n256), .c(new_n249), .d(new_n260), .o1(new_n261));
  inv000aa1d42x5               g166(.a(\b[22] ), .o1(new_n262));
  nanb02aa1n02x5               g167(.a(\a[23] ), .b(new_n262), .out0(new_n263));
  nand42aa1n03x5               g168(.a(\b[22] ), .b(\a[23] ), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(new_n263), .b(new_n264), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  xnbna2aa1n03x5               g171(.a(new_n266), .b(new_n258), .c(new_n261), .out0(\s[23] ));
  inv040aa1n06x5               g172(.a(new_n261), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n266), .b(new_n268), .c(new_n201), .d(new_n257), .o1(new_n269));
  inv000aa1d42x5               g174(.a(\a[24] ), .o1(new_n270));
  inv000aa1d42x5               g175(.a(\b[23] ), .o1(new_n271));
  nanp02aa1n02x5               g176(.a(new_n271), .b(new_n270), .o1(new_n272));
  tech160nm_finand02aa1n03p5x5 g177(.a(\b[23] ), .b(\a[24] ), .o1(new_n273));
  nanp02aa1n02x5               g178(.a(new_n272), .b(new_n273), .o1(new_n274));
  tech160nm_fiaoi012aa1n02p5x5 g179(.a(new_n274), .b(new_n269), .c(new_n263), .o1(new_n275));
  tech160nm_fiaoi012aa1n05x5   g180(.a(new_n265), .b(new_n258), .c(new_n261), .o1(new_n276));
  nano22aa1n03x5               g181(.a(new_n276), .b(new_n263), .c(new_n274), .out0(new_n277));
  norp02aa1n03x5               g182(.a(new_n275), .b(new_n277), .o1(\s[24] ));
  nor042aa1n02x5               g183(.a(\b[22] ), .b(\a[23] ), .o1(new_n279));
  nor022aa1n04x5               g184(.a(\b[23] ), .b(\a[24] ), .o1(new_n280));
  nanb03aa1n02x5               g185(.a(new_n280), .b(new_n273), .c(new_n264), .out0(new_n281));
  nona22aa1n03x5               g186(.a(new_n256), .b(new_n281), .c(new_n279), .out0(new_n282));
  nor042aa1n02x5               g187(.a(new_n236), .b(new_n282), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n212), .c(new_n141), .d(new_n209), .o1(new_n284));
  nona22aa1n09x5               g189(.a(new_n219), .b(new_n217), .c(new_n230), .out0(new_n285));
  oaoi03aa1n02x5               g190(.a(\a[22] ), .b(\b[21] ), .c(new_n246), .o1(new_n286));
  nano23aa1n02x4               g191(.a(new_n279), .b(new_n280), .c(new_n273), .d(new_n264), .out0(new_n287));
  aoi012aa1n02x5               g192(.a(new_n280), .b(new_n279), .c(new_n273), .o1(new_n288));
  aobi12aa1n09x5               g193(.a(new_n288), .b(new_n287), .c(new_n286), .out0(new_n289));
  aoai13aa1n06x5               g194(.a(new_n289), .b(new_n282), .c(new_n285), .d(new_n239), .o1(new_n290));
  inv000aa1n02x5               g195(.a(new_n290), .o1(new_n291));
  xnrc02aa1n12x5               g196(.a(\b[24] ), .b(\a[25] ), .out0(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  xnbna2aa1n03x5               g198(.a(new_n293), .b(new_n284), .c(new_n291), .out0(\s[25] ));
  nor042aa1n03x5               g199(.a(\b[24] ), .b(\a[25] ), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  aoai13aa1n03x5               g201(.a(new_n293), .b(new_n290), .c(new_n201), .d(new_n283), .o1(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[25] ), .b(\a[26] ), .out0(new_n298));
  aoi012aa1n03x5               g203(.a(new_n298), .b(new_n297), .c(new_n296), .o1(new_n299));
  tech160nm_fiaoi012aa1n03p5x5 g204(.a(new_n292), .b(new_n284), .c(new_n291), .o1(new_n300));
  nano22aa1n03x5               g205(.a(new_n300), .b(new_n296), .c(new_n298), .out0(new_n301));
  nor002aa1n02x5               g206(.a(new_n299), .b(new_n301), .o1(\s[26] ));
  nano23aa1n06x5               g207(.a(new_n260), .b(new_n281), .c(new_n263), .d(new_n243), .out0(new_n303));
  nor042aa1n06x5               g208(.a(new_n298), .b(new_n292), .o1(new_n304));
  nano22aa1n12x5               g209(.a(new_n236), .b(new_n303), .c(new_n304), .out0(new_n305));
  aoai13aa1n06x5               g210(.a(new_n305), .b(new_n212), .c(new_n141), .d(new_n209), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[26] ), .b(\b[25] ), .c(new_n296), .carry(new_n307));
  aobi12aa1n06x5               g212(.a(new_n307), .b(new_n290), .c(new_n304), .out0(new_n308));
  xorc02aa1n12x5               g213(.a(\a[27] ), .b(\b[26] ), .out0(new_n309));
  xnbna2aa1n03x5               g214(.a(new_n309), .b(new_n306), .c(new_n308), .out0(\s[27] ));
  norp02aa1n02x5               g215(.a(\b[26] ), .b(\a[27] ), .o1(new_n311));
  inv040aa1n03x5               g216(.a(new_n311), .o1(new_n312));
  nand22aa1n09x5               g217(.a(new_n240), .b(new_n303), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n304), .o1(new_n314));
  aoai13aa1n12x5               g219(.a(new_n307), .b(new_n314), .c(new_n313), .d(new_n289), .o1(new_n315));
  aoai13aa1n06x5               g220(.a(new_n309), .b(new_n315), .c(new_n201), .d(new_n305), .o1(new_n316));
  xnrc02aa1n12x5               g221(.a(\b[27] ), .b(\a[28] ), .out0(new_n317));
  tech160nm_fiaoi012aa1n05x5   g222(.a(new_n317), .b(new_n316), .c(new_n312), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n309), .o1(new_n319));
  tech160nm_fiaoi012aa1n03p5x5 g224(.a(new_n319), .b(new_n306), .c(new_n308), .o1(new_n320));
  nano22aa1n03x5               g225(.a(new_n320), .b(new_n312), .c(new_n317), .out0(new_n321));
  norp02aa1n03x5               g226(.a(new_n318), .b(new_n321), .o1(\s[28] ));
  norb02aa1n06x5               g227(.a(new_n309), .b(new_n317), .out0(new_n323));
  aoai13aa1n06x5               g228(.a(new_n323), .b(new_n315), .c(new_n201), .d(new_n305), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[28] ), .b(\b[27] ), .c(new_n312), .carry(new_n325));
  xnrc02aa1n02x5               g230(.a(\b[28] ), .b(\a[29] ), .out0(new_n326));
  tech160nm_fiaoi012aa1n05x5   g231(.a(new_n326), .b(new_n324), .c(new_n325), .o1(new_n327));
  inv000aa1d42x5               g232(.a(new_n323), .o1(new_n328));
  tech160nm_fiaoi012aa1n02p5x5 g233(.a(new_n328), .b(new_n306), .c(new_n308), .o1(new_n329));
  nano22aa1n03x5               g234(.a(new_n329), .b(new_n325), .c(new_n326), .out0(new_n330));
  norp02aa1n03x5               g235(.a(new_n327), .b(new_n330), .o1(\s[29] ));
  xorb03aa1n02x5               g236(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1d15x5               g237(.a(new_n309), .b(new_n326), .c(new_n317), .out0(new_n333));
  aoai13aa1n06x5               g238(.a(new_n333), .b(new_n315), .c(new_n201), .d(new_n305), .o1(new_n334));
  oao003aa1n02x5               g239(.a(\a[29] ), .b(\b[28] ), .c(new_n325), .carry(new_n335));
  xnrc02aa1n02x5               g240(.a(\b[29] ), .b(\a[30] ), .out0(new_n336));
  tech160nm_fiaoi012aa1n02p5x5 g241(.a(new_n336), .b(new_n334), .c(new_n335), .o1(new_n337));
  inv000aa1d42x5               g242(.a(new_n333), .o1(new_n338));
  tech160nm_fiaoi012aa1n02p5x5 g243(.a(new_n338), .b(new_n306), .c(new_n308), .o1(new_n339));
  nano22aa1n03x5               g244(.a(new_n339), .b(new_n335), .c(new_n336), .out0(new_n340));
  norp02aa1n03x5               g245(.a(new_n337), .b(new_n340), .o1(\s[30] ));
  nona32aa1n02x4               g246(.a(new_n309), .b(new_n336), .c(new_n326), .d(new_n317), .out0(new_n342));
  inv000aa1n02x5               g247(.a(new_n342), .o1(new_n343));
  aoai13aa1n06x5               g248(.a(new_n343), .b(new_n315), .c(new_n201), .d(new_n305), .o1(new_n344));
  oao003aa1n02x5               g249(.a(\a[30] ), .b(\b[29] ), .c(new_n335), .carry(new_n345));
  xnrc02aa1n02x5               g250(.a(\b[30] ), .b(\a[31] ), .out0(new_n346));
  tech160nm_fiaoi012aa1n02p5x5 g251(.a(new_n346), .b(new_n344), .c(new_n345), .o1(new_n347));
  tech160nm_fiaoi012aa1n02p5x5 g252(.a(new_n342), .b(new_n306), .c(new_n308), .o1(new_n348));
  nano22aa1n03x5               g253(.a(new_n348), .b(new_n345), .c(new_n346), .out0(new_n349));
  norp02aa1n03x5               g254(.a(new_n347), .b(new_n349), .o1(\s[31] ));
  xnbna2aa1n03x5               g255(.a(new_n101), .b(new_n133), .c(new_n102), .out0(\s[3] ));
  aoi013aa1n02x4               g256(.a(new_n99), .b(new_n133), .c(new_n102), .d(new_n100), .o1(new_n352));
  oaib12aa1n02x5               g257(.a(new_n109), .b(new_n352), .c(new_n134), .out0(\s[4] ));
  xnbna2aa1n03x5               g258(.a(new_n115), .b(new_n109), .c(new_n106), .out0(\s[5] ));
  inv000aa1d42x5               g259(.a(new_n114), .o1(new_n355));
  norb02aa1n02x5               g260(.a(new_n110), .b(new_n123), .out0(new_n356));
  nanb03aa1n02x5               g261(.a(new_n115), .b(new_n109), .c(new_n106), .out0(new_n357));
  xnbna2aa1n03x5               g262(.a(new_n356), .b(new_n357), .c(new_n355), .out0(\s[6] ));
  nanp03aa1n02x5               g263(.a(new_n357), .b(new_n355), .c(new_n356), .o1(new_n359));
  xorc02aa1n02x5               g264(.a(\a[7] ), .b(\b[6] ), .out0(new_n360));
  xobna2aa1n03x5               g265(.a(new_n360), .b(new_n359), .c(new_n110), .out0(\s[7] ));
  nanp03aa1n02x5               g266(.a(new_n359), .b(new_n110), .c(new_n360), .o1(new_n362));
  oai012aa1n02x5               g267(.a(new_n362), .b(\b[6] ), .c(\a[7] ), .o1(new_n363));
  xorb03aa1n02x5               g268(.a(new_n363), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g269(.a(new_n188), .b(new_n126), .c(new_n98), .out0(\s[9] ));
endmodule


