// Benchmark "adder" written by ABC on Thu Jul 18 01:33:40 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n172, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n178, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n219, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n227, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n274, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n354, new_n356, new_n359, new_n360, new_n362,
    new_n363, new_n364, new_n366, new_n368, new_n369;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[2] ), .o1(new_n97));
  inv020aa1d32x5               g002(.a(\b[1] ), .o1(new_n98));
  tech160nm_finand02aa1n03p5x5 g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nand22aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand02aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aob012aa1n06x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  nor002aa1d32x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand22aa1n12x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norb02aa1n03x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nor002aa1d32x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand02aa1d16x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  norb02aa1n06x5               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  nand23aa1n06x5               g013(.a(new_n102), .b(new_n105), .c(new_n108), .o1(new_n109));
  aoi012aa1n09x5               g014(.a(new_n103), .b(new_n106), .c(new_n104), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand02aa1d28x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1d28x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n09x5               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  nor042aa1d18x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nand02aa1d28x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  norb02aa1n12x5               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  xorc02aa1n12x5               g023(.a(\a[5] ), .b(\b[4] ), .out0(new_n119));
  nand23aa1n03x5               g024(.a(new_n115), .b(new_n118), .c(new_n119), .o1(new_n120));
  inv040aa1n04x5               g025(.a(new_n116), .o1(new_n121));
  nor042aa1d18x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  aob012aa1d18x5               g027(.a(new_n121), .b(new_n122), .c(new_n117), .out0(new_n123));
  aoi012aa1n12x5               g028(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n124));
  aobi12aa1n06x5               g029(.a(new_n124), .b(new_n115), .c(new_n123), .out0(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n120), .c(new_n109), .d(new_n110), .o1(new_n126));
  nor042aa1n04x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nanp02aa1n09x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  nor042aa1n03x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanp02aa1n09x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nanb02aa1n02x5               g036(.a(new_n130), .b(new_n131), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n127), .c(new_n126), .d(new_n128), .o1(new_n133));
  nona22aa1n02x4               g038(.a(new_n131), .b(new_n127), .c(new_n130), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n133), .b(new_n134), .c(new_n129), .d(new_n126), .o1(\s[10] ));
  tech160nm_fioaoi03aa1n05x5   g040(.a(new_n97), .b(new_n98), .c(new_n100), .o1(new_n136));
  nona23aa1n09x5               g041(.a(new_n107), .b(new_n104), .c(new_n103), .d(new_n106), .out0(new_n137));
  oai012aa1n12x5               g042(.a(new_n110), .b(new_n137), .c(new_n136), .o1(new_n138));
  nona23aa1n09x5               g043(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n139));
  inv040aa1n02x5               g044(.a(new_n118), .o1(new_n140));
  xnrc02aa1n02x5               g045(.a(\b[4] ), .b(\a[5] ), .out0(new_n141));
  nor043aa1n02x5               g046(.a(new_n139), .b(new_n140), .c(new_n141), .o1(new_n142));
  oaib12aa1n09x5               g047(.a(new_n124), .b(new_n139), .c(new_n123), .out0(new_n143));
  nano23aa1n06x5               g048(.a(new_n130), .b(new_n127), .c(new_n128), .d(new_n131), .out0(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n143), .c(new_n138), .d(new_n142), .o1(new_n145));
  oai012aa1n02x5               g050(.a(new_n131), .b(new_n127), .c(new_n130), .o1(new_n146));
  xorc02aa1n12x5               g051(.a(\a[11] ), .b(\b[10] ), .out0(new_n147));
  xnbna2aa1n03x5               g052(.a(new_n147), .b(new_n145), .c(new_n146), .out0(\s[11] ));
  nor002aa1n16x5               g053(.a(\b[10] ), .b(\a[11] ), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aob012aa1n02x5               g055(.a(new_n147), .b(new_n145), .c(new_n146), .out0(new_n151));
  nor002aa1n04x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nand42aa1n08x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  norb02aa1n09x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  nona23aa1n02x4               g059(.a(new_n151), .b(new_n153), .c(new_n152), .d(new_n149), .out0(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n154), .c(new_n150), .d(new_n151), .o1(\s[12] ));
  nanp03aa1n02x5               g061(.a(new_n144), .b(new_n147), .c(new_n154), .o1(new_n157));
  nanb02aa1n02x5               g062(.a(new_n157), .b(new_n126), .out0(new_n158));
  nanp02aa1n03x5               g063(.a(new_n138), .b(new_n142), .o1(new_n159));
  inv040aa1n02x5               g064(.a(new_n152), .o1(new_n160));
  oai022aa1d18x5               g065(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n161));
  aoi022aa1d24x5               g066(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n162));
  aoai13aa1n12x5               g067(.a(new_n153), .b(new_n149), .c(new_n161), .d(new_n162), .o1(new_n163));
  nand22aa1n12x5               g068(.a(new_n163), .b(new_n160), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n04x5               g070(.a(new_n165), .b(new_n157), .c(new_n159), .d(new_n125), .o1(new_n166));
  nor042aa1d18x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  nand42aa1d28x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  nano22aa1n02x4               g074(.a(new_n169), .b(new_n163), .c(new_n160), .out0(new_n170));
  aoi022aa1n02x5               g075(.a(new_n166), .b(new_n169), .c(new_n158), .d(new_n170), .o1(\s[13] ));
  inv000aa1d42x5               g076(.a(new_n167), .o1(new_n172));
  nand22aa1n03x5               g077(.a(new_n166), .b(new_n169), .o1(new_n173));
  nor042aa1n06x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nand42aa1d28x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  oai022aa1d18x5               g081(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n177));
  nanb03aa1n02x5               g082(.a(new_n177), .b(new_n173), .c(new_n175), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n176), .c(new_n172), .d(new_n173), .o1(\s[14] ));
  nano23aa1d15x5               g084(.a(new_n167), .b(new_n174), .c(new_n175), .d(new_n168), .out0(new_n180));
  oaoi03aa1n02x5               g085(.a(\a[14] ), .b(\b[13] ), .c(new_n172), .o1(new_n181));
  nor022aa1n08x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nand42aa1d28x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  norb02aa1n02x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  aoai13aa1n06x5               g089(.a(new_n184), .b(new_n181), .c(new_n166), .d(new_n180), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n184), .b(new_n181), .c(new_n166), .d(new_n180), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n185), .b(new_n186), .out0(\s[15] ));
  inv000aa1n02x5               g092(.a(new_n182), .o1(new_n188));
  nor002aa1n20x5               g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  nand02aa1d28x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  inv000aa1d42x5               g096(.a(new_n190), .o1(new_n192));
  oai022aa1n02x5               g097(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n193));
  nona22aa1n03x5               g098(.a(new_n185), .b(new_n192), .c(new_n193), .out0(new_n194));
  aoai13aa1n03x5               g099(.a(new_n194), .b(new_n191), .c(new_n188), .d(new_n185), .o1(\s[16] ));
  nano23aa1n09x5               g100(.a(new_n182), .b(new_n189), .c(new_n190), .d(new_n183), .out0(new_n196));
  nand22aa1n06x5               g101(.a(new_n196), .b(new_n180), .o1(new_n197));
  nano32aa1n03x7               g102(.a(new_n197), .b(new_n144), .c(new_n147), .d(new_n154), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n143), .c(new_n138), .d(new_n142), .o1(new_n199));
  inv000aa1d42x5               g104(.a(new_n189), .o1(new_n200));
  nanp03aa1n02x5               g105(.a(new_n177), .b(new_n175), .c(new_n183), .o1(new_n201));
  aoai13aa1n03x5               g106(.a(new_n200), .b(new_n192), .c(new_n201), .d(new_n188), .o1(new_n202));
  aoib12aa1n12x5               g107(.a(new_n202), .b(new_n164), .c(new_n197), .out0(new_n203));
  nanp02aa1n09x5               g108(.a(new_n199), .b(new_n203), .o1(new_n204));
  nor002aa1d32x5               g109(.a(\b[16] ), .b(\a[17] ), .o1(new_n205));
  nand42aa1d28x5               g110(.a(\b[16] ), .b(\a[17] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  nanp02aa1n03x5               g112(.a(new_n201), .b(new_n188), .o1(new_n208));
  aoi112aa1n02x5               g113(.a(new_n207), .b(new_n189), .c(new_n208), .d(new_n190), .o1(new_n209));
  oa0012aa1n02x5               g114(.a(new_n209), .b(new_n165), .c(new_n197), .o(new_n210));
  aoi022aa1n02x5               g115(.a(new_n204), .b(new_n207), .c(new_n210), .d(new_n199), .o1(\s[17] ));
  inv000aa1d42x5               g116(.a(new_n205), .o1(new_n212));
  aoi012aa1n02x5               g117(.a(new_n189), .b(new_n208), .c(new_n190), .o1(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n197), .c(new_n160), .d(new_n163), .o1(new_n214));
  aoai13aa1n02x5               g119(.a(new_n207), .b(new_n214), .c(new_n126), .d(new_n198), .o1(new_n215));
  nor002aa1d32x5               g120(.a(\b[17] ), .b(\a[18] ), .o1(new_n216));
  nand42aa1d28x5               g121(.a(\b[17] ), .b(\a[18] ), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(new_n218));
  nona23aa1n02x4               g123(.a(new_n215), .b(new_n217), .c(new_n216), .d(new_n205), .out0(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n218), .c(new_n212), .d(new_n215), .o1(\s[18] ));
  nano23aa1d15x5               g125(.a(new_n205), .b(new_n216), .c(new_n217), .d(new_n206), .out0(new_n221));
  oaoi03aa1n02x5               g126(.a(\a[18] ), .b(\b[17] ), .c(new_n212), .o1(new_n222));
  nor042aa1d18x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  nand22aa1n04x5               g128(.a(\b[18] ), .b(\a[19] ), .o1(new_n224));
  norb02aa1n09x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n222), .c(new_n204), .d(new_n221), .o1(new_n226));
  aoi112aa1n02x5               g131(.a(new_n225), .b(new_n222), .c(new_n204), .d(new_n221), .o1(new_n227));
  norb02aa1n03x4               g132(.a(new_n226), .b(new_n227), .out0(\s[19] ));
  xnrc02aa1n02x5               g133(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g134(.a(new_n223), .o1(new_n230));
  nor042aa1n12x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  nand22aa1n12x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  norb02aa1n06x5               g137(.a(new_n232), .b(new_n231), .out0(new_n233));
  norb03aa1n02x5               g138(.a(new_n232), .b(new_n223), .c(new_n231), .out0(new_n234));
  tech160nm_finand02aa1n03p5x5 g139(.a(new_n226), .b(new_n234), .o1(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n233), .c(new_n230), .d(new_n226), .o1(\s[20] ));
  nand23aa1d12x5               g141(.a(new_n221), .b(new_n225), .c(new_n233), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  nanb03aa1n06x5               g143(.a(new_n231), .b(new_n232), .c(new_n224), .out0(new_n239));
  oai112aa1n06x5               g144(.a(new_n230), .b(new_n217), .c(new_n216), .d(new_n205), .o1(new_n240));
  aoi012aa1d24x5               g145(.a(new_n231), .b(new_n223), .c(new_n232), .o1(new_n241));
  oai012aa1n06x5               g146(.a(new_n241), .b(new_n240), .c(new_n239), .o1(new_n242));
  nor002aa1d24x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  nand22aa1n03x5               g148(.a(\b[20] ), .b(\a[21] ), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n242), .c(new_n204), .d(new_n238), .o1(new_n246));
  nano22aa1n03x7               g151(.a(new_n231), .b(new_n224), .c(new_n232), .out0(new_n247));
  tech160nm_fioai012aa1n03p5x5 g152(.a(new_n217), .b(\b[18] ), .c(\a[19] ), .o1(new_n248));
  oab012aa1n06x5               g153(.a(new_n248), .b(new_n205), .c(new_n216), .out0(new_n249));
  inv020aa1n02x5               g154(.a(new_n241), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(new_n250), .b(new_n245), .c(new_n249), .d(new_n247), .o1(new_n251));
  aobi12aa1n02x5               g156(.a(new_n251), .b(new_n204), .c(new_n238), .out0(new_n252));
  norb02aa1n03x4               g157(.a(new_n246), .b(new_n252), .out0(\s[21] ));
  inv000aa1d42x5               g158(.a(new_n243), .o1(new_n254));
  nor002aa1n08x5               g159(.a(\b[21] ), .b(\a[22] ), .o1(new_n255));
  nand02aa1d28x5               g160(.a(\b[21] ), .b(\a[22] ), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n256), .b(new_n255), .out0(new_n257));
  norb03aa1n02x5               g162(.a(new_n256), .b(new_n243), .c(new_n255), .out0(new_n258));
  tech160nm_finand02aa1n03p5x5 g163(.a(new_n246), .b(new_n258), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n259), .b(new_n257), .c(new_n254), .d(new_n246), .o1(\s[22] ));
  nano23aa1n06x5               g165(.a(new_n243), .b(new_n255), .c(new_n256), .d(new_n244), .out0(new_n261));
  norb02aa1n03x5               g166(.a(new_n261), .b(new_n237), .out0(new_n262));
  aoai13aa1n06x5               g167(.a(new_n261), .b(new_n250), .c(new_n249), .d(new_n247), .o1(new_n263));
  aoi012aa1d24x5               g168(.a(new_n255), .b(new_n243), .c(new_n256), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(new_n263), .b(new_n264), .o1(new_n265));
  nor002aa1d32x5               g170(.a(\b[22] ), .b(\a[23] ), .o1(new_n266));
  nand42aa1n04x5               g171(.a(\b[22] ), .b(\a[23] ), .o1(new_n267));
  norb02aa1n02x5               g172(.a(new_n267), .b(new_n266), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n265), .c(new_n204), .d(new_n262), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n264), .o1(new_n270));
  nona22aa1n02x4               g175(.a(new_n263), .b(new_n270), .c(new_n268), .out0(new_n271));
  aoi012aa1n02x5               g176(.a(new_n271), .b(new_n204), .c(new_n262), .o1(new_n272));
  norb02aa1n03x4               g177(.a(new_n269), .b(new_n272), .out0(\s[23] ));
  inv000aa1d42x5               g178(.a(new_n266), .o1(new_n274));
  nor042aa1n06x5               g179(.a(\b[23] ), .b(\a[24] ), .o1(new_n275));
  and002aa1n18x5               g180(.a(\b[23] ), .b(\a[24] ), .o(new_n276));
  norp02aa1n02x5               g181(.a(new_n276), .b(new_n275), .o1(new_n277));
  norp03aa1n02x5               g182(.a(new_n276), .b(new_n275), .c(new_n266), .o1(new_n278));
  tech160nm_finand02aa1n03p5x5 g183(.a(new_n269), .b(new_n278), .o1(new_n279));
  aoai13aa1n03x5               g184(.a(new_n279), .b(new_n277), .c(new_n274), .d(new_n269), .o1(\s[24] ));
  nano32aa1n03x7               g185(.a(new_n237), .b(new_n277), .c(new_n261), .d(new_n268), .out0(new_n281));
  nano23aa1d18x5               g186(.a(new_n276), .b(new_n275), .c(new_n274), .d(new_n267), .out0(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  oab012aa1d15x5               g188(.a(new_n275), .b(new_n274), .c(new_n276), .out0(new_n284));
  aoai13aa1n06x5               g189(.a(new_n284), .b(new_n283), .c(new_n263), .d(new_n264), .o1(new_n285));
  nor042aa1n04x5               g190(.a(\b[24] ), .b(\a[25] ), .o1(new_n286));
  nand42aa1n02x5               g191(.a(\b[24] ), .b(\a[25] ), .o1(new_n287));
  norb02aa1n02x5               g192(.a(new_n287), .b(new_n286), .out0(new_n288));
  aoai13aa1n06x5               g193(.a(new_n288), .b(new_n285), .c(new_n204), .d(new_n281), .o1(new_n289));
  aoai13aa1n04x5               g194(.a(new_n282), .b(new_n270), .c(new_n242), .d(new_n261), .o1(new_n290));
  nanb03aa1n02x5               g195(.a(new_n288), .b(new_n290), .c(new_n284), .out0(new_n291));
  aoi012aa1n02x5               g196(.a(new_n291), .b(new_n204), .c(new_n281), .o1(new_n292));
  norb02aa1n03x4               g197(.a(new_n289), .b(new_n292), .out0(\s[25] ));
  inv040aa1n03x5               g198(.a(new_n286), .o1(new_n294));
  nor002aa1n03x5               g199(.a(\b[25] ), .b(\a[26] ), .o1(new_n295));
  and002aa1n06x5               g200(.a(\b[25] ), .b(\a[26] ), .o(new_n296));
  norp02aa1n02x5               g201(.a(new_n296), .b(new_n295), .o1(new_n297));
  norp03aa1n02x5               g202(.a(new_n296), .b(new_n295), .c(new_n286), .o1(new_n298));
  tech160nm_finand02aa1n03p5x5 g203(.a(new_n289), .b(new_n298), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n297), .c(new_n294), .d(new_n289), .o1(\s[26] ));
  nano23aa1n06x5               g205(.a(new_n296), .b(new_n295), .c(new_n294), .d(new_n287), .out0(new_n301));
  inv040aa1n02x5               g206(.a(new_n301), .o1(new_n302));
  nano23aa1n06x5               g207(.a(new_n302), .b(new_n237), .c(new_n282), .d(new_n261), .out0(new_n303));
  aoai13aa1n06x5               g208(.a(new_n303), .b(new_n214), .c(new_n126), .d(new_n198), .o1(new_n304));
  aobi12aa1n06x5               g209(.a(new_n303), .b(new_n199), .c(new_n203), .out0(new_n305));
  oab012aa1n02x4               g210(.a(new_n295), .b(new_n294), .c(new_n296), .out0(new_n306));
  aoai13aa1n04x5               g211(.a(new_n306), .b(new_n302), .c(new_n290), .d(new_n284), .o1(new_n307));
  xorc02aa1n12x5               g212(.a(\a[27] ), .b(\b[26] ), .out0(new_n308));
  oaih12aa1n02x5               g213(.a(new_n308), .b(new_n307), .c(new_n305), .o1(new_n309));
  inv000aa1n02x5               g214(.a(new_n306), .o1(new_n310));
  aoi112aa1n02x5               g215(.a(new_n308), .b(new_n310), .c(new_n285), .d(new_n301), .o1(new_n311));
  aobi12aa1n02x7               g216(.a(new_n309), .b(new_n311), .c(new_n304), .out0(\s[27] ));
  norp02aa1n02x5               g217(.a(\b[26] ), .b(\a[27] ), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n313), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .out0(new_n315));
  aoi012aa1n06x5               g220(.a(new_n310), .b(new_n285), .c(new_n301), .o1(new_n316));
  inv030aa1n02x5               g221(.a(new_n308), .o1(new_n317));
  oai022aa1n02x5               g222(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n318));
  aoi012aa1n02x5               g223(.a(new_n318), .b(\a[28] ), .c(\b[27] ), .o1(new_n319));
  aoai13aa1n04x5               g224(.a(new_n319), .b(new_n317), .c(new_n316), .d(new_n304), .o1(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n315), .c(new_n309), .d(new_n314), .o1(\s[28] ));
  and002aa1n02x5               g226(.a(new_n315), .b(new_n308), .o(new_n322));
  oaih12aa1n02x5               g227(.a(new_n322), .b(new_n307), .c(new_n305), .o1(new_n323));
  xorc02aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .out0(new_n324));
  aob012aa1n02x5               g229(.a(new_n318), .b(\b[27] ), .c(\a[28] ), .out0(new_n325));
  norb02aa1n02x5               g230(.a(new_n325), .b(new_n324), .out0(new_n326));
  inv000aa1d42x5               g231(.a(new_n322), .o1(new_n327));
  aoai13aa1n02x7               g232(.a(new_n325), .b(new_n327), .c(new_n316), .d(new_n304), .o1(new_n328));
  aoi022aa1n03x5               g233(.a(new_n328), .b(new_n324), .c(new_n323), .d(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g234(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g235(.a(new_n317), .b(new_n315), .c(new_n324), .out0(new_n331));
  oaih12aa1n02x5               g236(.a(new_n331), .b(new_n307), .c(new_n305), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[30] ), .b(\b[29] ), .out0(new_n333));
  inv000aa1d42x5               g238(.a(\a[29] ), .o1(new_n334));
  inv000aa1d42x5               g239(.a(\b[28] ), .o1(new_n335));
  oaib12aa1n02x5               g240(.a(new_n325), .b(\b[28] ), .c(new_n334), .out0(new_n336));
  oaoi13aa1n02x5               g241(.a(new_n333), .b(new_n336), .c(new_n334), .d(new_n335), .o1(new_n337));
  inv000aa1n02x5               g242(.a(new_n331), .o1(new_n338));
  oaib12aa1n02x5               g243(.a(new_n336), .b(new_n335), .c(\a[29] ), .out0(new_n339));
  aoai13aa1n03x5               g244(.a(new_n339), .b(new_n338), .c(new_n316), .d(new_n304), .o1(new_n340));
  aoi022aa1n03x5               g245(.a(new_n340), .b(new_n333), .c(new_n332), .d(new_n337), .o1(\s[30] ));
  nano32aa1n02x4               g246(.a(new_n317), .b(new_n333), .c(new_n315), .d(new_n324), .out0(new_n342));
  oaih12aa1n02x5               g247(.a(new_n342), .b(new_n307), .c(new_n305), .o1(new_n343));
  aoi022aa1n02x5               g248(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n344));
  norb02aa1n02x5               g249(.a(\b[30] ), .b(\a[31] ), .out0(new_n345));
  obai22aa1n02x7               g250(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n346));
  aoi112aa1n02x5               g251(.a(new_n346), .b(new_n345), .c(new_n336), .d(new_n344), .o1(new_n347));
  xorc02aa1n02x5               g252(.a(\a[31] ), .b(\b[30] ), .out0(new_n348));
  inv000aa1n02x5               g253(.a(new_n342), .o1(new_n349));
  norp02aa1n02x5               g254(.a(\b[29] ), .b(\a[30] ), .o1(new_n350));
  aoi012aa1n02x5               g255(.a(new_n350), .b(new_n336), .c(new_n344), .o1(new_n351));
  aoai13aa1n03x5               g256(.a(new_n351), .b(new_n349), .c(new_n316), .d(new_n304), .o1(new_n352));
  aoi022aa1n03x5               g257(.a(new_n352), .b(new_n348), .c(new_n343), .d(new_n347), .o1(\s[31] ));
  nanp03aa1n02x5               g258(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n108), .b(new_n354), .c(new_n99), .out0(\s[3] ));
  aoi112aa1n02x5               g260(.a(new_n106), .b(new_n105), .c(new_n102), .d(new_n108), .o1(new_n356));
  aoib12aa1n02x5               g261(.a(new_n356), .b(new_n138), .c(new_n103), .out0(\s[4] ));
  xnbna2aa1n03x5               g262(.a(new_n119), .b(new_n109), .c(new_n110), .out0(\s[5] ));
  aoai13aa1n02x5               g263(.a(new_n118), .b(new_n122), .c(new_n138), .d(new_n119), .o1(new_n359));
  aoi112aa1n02x5               g264(.a(new_n122), .b(new_n118), .c(new_n138), .d(new_n119), .o1(new_n360));
  norb02aa1n02x5               g265(.a(new_n359), .b(new_n360), .out0(\s[6] ));
  aboi22aa1n03x5               g266(.a(new_n113), .b(new_n114), .c(new_n359), .d(new_n121), .out0(new_n362));
  norb03aa1n02x5               g267(.a(new_n114), .b(new_n113), .c(new_n116), .out0(new_n363));
  nanp02aa1n02x5               g268(.a(new_n359), .b(new_n363), .o1(new_n364));
  nanb02aa1n02x5               g269(.a(new_n362), .b(new_n364), .out0(\s[7] ));
  nanb02aa1n02x5               g270(.a(new_n111), .b(new_n112), .out0(new_n366));
  xnbna2aa1n03x5               g271(.a(new_n366), .b(new_n364), .c(new_n114), .out0(\s[8] ));
  oaib12aa1n02x5               g272(.a(new_n124), .b(new_n127), .c(new_n128), .out0(new_n368));
  aoi012aa1n02x5               g273(.a(new_n368), .b(new_n115), .c(new_n123), .o1(new_n369));
  aoi022aa1n02x5               g274(.a(new_n126), .b(new_n129), .c(new_n159), .d(new_n369), .o1(\s[9] ));
endmodule


