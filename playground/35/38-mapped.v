// Benchmark "adder" written by ABC on Thu Jul 18 06:14:29 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n321, new_n322, new_n325, new_n326,
    new_n327, new_n329, new_n331, new_n332, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\a[2] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[1] ), .o1(new_n99));
  nand02aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  oao003aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .carry(new_n101));
  xorc02aa1n02x5               g006(.a(\a[3] ), .b(\b[2] ), .out0(new_n102));
  xorc02aa1n02x5               g007(.a(\a[4] ), .b(\b[3] ), .out0(new_n103));
  nanp03aa1n02x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  aoi112aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n105));
  oab012aa1n02x4               g010(.a(new_n105), .b(\a[4] ), .c(\b[3] ), .out0(new_n106));
  xnrc02aa1n02x5               g011(.a(\b[6] ), .b(\a[7] ), .out0(new_n107));
  xnrc02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .out0(new_n108));
  tech160nm_fixorc02aa1n02p5x5 g013(.a(\a[5] ), .b(\b[4] ), .out0(new_n109));
  xorc02aa1n12x5               g014(.a(\a[8] ), .b(\b[7] ), .out0(new_n110));
  nona23aa1n02x4               g015(.a(new_n109), .b(new_n110), .c(new_n108), .d(new_n107), .out0(new_n111));
  and002aa1n03x5               g016(.a(\b[6] ), .b(\a[7] ), .o(new_n112));
  norp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  aoi112aa1n03x5               g018(.a(new_n112), .b(new_n113), .c(\a[6] ), .d(\b[5] ), .o1(new_n114));
  oai022aa1n06x5               g019(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n115));
  inv000aa1d42x5               g020(.a(\a[7] ), .o1(new_n116));
  nanb02aa1n02x5               g021(.a(\b[6] ), .b(new_n116), .out0(new_n117));
  oaoi03aa1n02x5               g022(.a(\a[8] ), .b(\b[7] ), .c(new_n117), .o1(new_n118));
  aoi013aa1n09x5               g023(.a(new_n118), .b(new_n114), .c(new_n110), .d(new_n115), .o1(new_n119));
  aoai13aa1n06x5               g024(.a(new_n119), .b(new_n111), .c(new_n104), .d(new_n106), .o1(new_n120));
  tech160nm_fixorc02aa1n03p5x5 g025(.a(\a[9] ), .b(\b[8] ), .out0(new_n121));
  norp02aa1n02x5               g026(.a(\b[9] ), .b(\a[10] ), .o1(new_n122));
  nand42aa1n03x5               g027(.a(\b[9] ), .b(\a[10] ), .o1(new_n123));
  nanb02aa1n02x5               g028(.a(new_n122), .b(new_n123), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n97), .c(new_n120), .d(new_n121), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(new_n104), .b(new_n106), .o1(new_n126));
  xorc02aa1n02x5               g031(.a(\a[6] ), .b(\b[5] ), .out0(new_n127));
  nano32aa1n02x4               g032(.a(new_n107), .b(new_n110), .c(new_n127), .d(new_n109), .out0(new_n128));
  inv000aa1d42x5               g033(.a(new_n119), .o1(new_n129));
  aoai13aa1n02x5               g034(.a(new_n121), .b(new_n129), .c(new_n126), .d(new_n128), .o1(new_n130));
  nona22aa1n02x4               g035(.a(new_n130), .b(new_n124), .c(new_n97), .out0(new_n131));
  nanp02aa1n02x5               g036(.a(new_n125), .b(new_n131), .o1(\s[10] ));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nano22aa1n02x4               g039(.a(new_n134), .b(new_n123), .c(new_n133), .out0(new_n135));
  nanb02aa1n03x5               g040(.a(new_n134), .b(new_n133), .out0(new_n136));
  nanp02aa1n02x5               g041(.a(new_n131), .b(new_n123), .o1(new_n137));
  aoi022aa1n02x5               g042(.a(new_n137), .b(new_n136), .c(new_n131), .d(new_n135), .o1(\s[11] ));
  nanp02aa1n02x5               g043(.a(new_n131), .b(new_n135), .o1(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n06x4               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  aoib12aa1n02x5               g047(.a(new_n134), .b(new_n141), .c(new_n140), .out0(new_n143));
  oai012aa1n02x5               g048(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .o1(new_n144));
  aoi022aa1n02x5               g049(.a(new_n144), .b(new_n142), .c(new_n139), .d(new_n143), .o1(\s[12] ));
  nona23aa1d18x5               g050(.a(new_n121), .b(new_n142), .c(new_n136), .d(new_n124), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n129), .c(new_n126), .d(new_n128), .o1(new_n148));
  nano23aa1n02x4               g053(.a(new_n140), .b(new_n134), .c(new_n141), .d(new_n133), .out0(new_n149));
  oai122aa1n02x7               g054(.a(new_n123), .b(new_n122), .c(new_n97), .d(\b[10] ), .e(\a[11] ), .o1(new_n150));
  inv000aa1n02x5               g055(.a(new_n150), .o1(new_n151));
  tech160nm_fiao0012aa1n02p5x5 g056(.a(new_n140), .b(new_n134), .c(new_n141), .o(new_n152));
  tech160nm_fiao0012aa1n02p5x5 g057(.a(new_n152), .b(new_n151), .c(new_n149), .o(new_n153));
  nanb02aa1n03x5               g058(.a(new_n153), .b(new_n148), .out0(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nand42aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  nor022aa1n08x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nand42aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(new_n159), .b(new_n158), .c(new_n154), .d(new_n160), .o1(new_n161));
  inv000aa1d42x5               g066(.a(new_n159), .o1(new_n162));
  aob012aa1n02x5               g067(.a(new_n162), .b(new_n154), .c(new_n160), .out0(new_n163));
  aoi012aa1n02x5               g068(.a(new_n161), .b(new_n163), .c(new_n158), .o1(\s[14] ));
  nona23aa1d18x5               g069(.a(new_n157), .b(new_n160), .c(new_n159), .d(new_n156), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n153), .c(new_n120), .d(new_n147), .o1(new_n167));
  oaoi03aa1n02x5               g072(.a(\a[14] ), .b(\b[13] ), .c(new_n162), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  orn002aa1n02x7               g074(.a(\a[15] ), .b(\b[14] ), .o(new_n170));
  nanp02aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand02aa1n04x5               g076(.a(new_n170), .b(new_n171), .o1(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n167), .c(new_n169), .out0(\s[15] ));
  aoai13aa1n03x5               g079(.a(new_n173), .b(new_n168), .c(new_n154), .d(new_n166), .o1(new_n175));
  tech160nm_fixnrc02aa1n04x5   g080(.a(\b[15] ), .b(\a[16] ), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  and002aa1n02x5               g082(.a(new_n176), .b(new_n170), .o(new_n178));
  aoai13aa1n02x5               g083(.a(new_n170), .b(new_n172), .c(new_n167), .d(new_n169), .o1(new_n179));
  aoi022aa1n03x5               g084(.a(new_n179), .b(new_n177), .c(new_n175), .d(new_n178), .o1(\s[16] ));
  norp03aa1d12x5               g085(.a(new_n165), .b(new_n172), .c(new_n176), .o1(new_n181));
  norb02aa1d21x5               g086(.a(new_n181), .b(new_n146), .out0(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n129), .c(new_n126), .d(new_n128), .o1(new_n183));
  aoai13aa1n06x5               g088(.a(new_n181), .b(new_n152), .c(new_n151), .d(new_n149), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\a[16] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\b[15] ), .o1(new_n186));
  oai112aa1n02x5               g091(.a(new_n157), .b(new_n171), .c(new_n156), .d(new_n159), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(new_n187), .b(new_n170), .o1(new_n188));
  oaoi03aa1n02x5               g093(.a(new_n185), .b(new_n186), .c(new_n188), .o1(new_n189));
  nanp02aa1n06x5               g094(.a(new_n184), .b(new_n189), .o1(new_n190));
  inv040aa1n06x5               g095(.a(new_n190), .o1(new_n191));
  xorc02aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  xnbna2aa1n03x5               g097(.a(new_n192), .b(new_n191), .c(new_n183), .out0(\s[17] ));
  inv000aa1d42x5               g098(.a(\a[17] ), .o1(new_n194));
  nanb02aa1n02x5               g099(.a(\b[16] ), .b(new_n194), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n192), .b(new_n190), .c(new_n120), .d(new_n182), .o1(new_n196));
  xorc02aa1n02x5               g101(.a(\a[18] ), .b(\b[17] ), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n196), .c(new_n195), .out0(\s[18] ));
  inv000aa1d42x5               g103(.a(\a[18] ), .o1(new_n199));
  xroi22aa1d04x5               g104(.a(new_n194), .b(\b[16] ), .c(new_n199), .d(\b[17] ), .out0(new_n200));
  aoai13aa1n03x5               g105(.a(new_n200), .b(new_n190), .c(new_n120), .d(new_n182), .o1(new_n201));
  oai022aa1n02x5               g106(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n202));
  oaib12aa1n06x5               g107(.a(new_n202), .b(new_n199), .c(\b[17] ), .out0(new_n203));
  nor042aa1n09x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nand02aa1n03x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(new_n204), .b(new_n205), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n201), .c(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n09x5               g114(.a(new_n191), .b(new_n183), .o1(new_n210));
  oaoi03aa1n02x5               g115(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n211));
  aoai13aa1n03x5               g116(.a(new_n207), .b(new_n211), .c(new_n210), .d(new_n200), .o1(new_n212));
  nor042aa1n04x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand02aa1n06x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  aoib12aa1n02x5               g120(.a(new_n204), .b(new_n214), .c(new_n213), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n204), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n206), .c(new_n201), .d(new_n203), .o1(new_n218));
  aoi022aa1n02x5               g123(.a(new_n218), .b(new_n215), .c(new_n212), .d(new_n216), .o1(\s[20] ));
  nona23aa1n09x5               g124(.a(new_n214), .b(new_n205), .c(new_n204), .d(new_n213), .out0(new_n220));
  nano22aa1n02x5               g125(.a(new_n220), .b(new_n192), .c(new_n197), .out0(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n190), .c(new_n120), .d(new_n182), .o1(new_n222));
  aoi012aa1n12x5               g127(.a(new_n213), .b(new_n204), .c(new_n214), .o1(new_n223));
  oai012aa1d24x5               g128(.a(new_n223), .b(new_n220), .c(new_n203), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[20] ), .b(\a[21] ), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n222), .c(new_n225), .out0(\s[21] ));
  aoai13aa1n03x5               g133(.a(new_n227), .b(new_n224), .c(new_n210), .d(new_n221), .o1(new_n229));
  xnrc02aa1n12x5               g134(.a(\b[21] ), .b(\a[22] ), .out0(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nor042aa1n03x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n230), .b(new_n232), .out0(new_n233));
  inv000aa1n03x5               g138(.a(new_n232), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n226), .c(new_n222), .d(new_n225), .o1(new_n235));
  aoi022aa1n02x5               g140(.a(new_n235), .b(new_n231), .c(new_n229), .d(new_n233), .o1(\s[22] ));
  nano23aa1n06x5               g141(.a(new_n204), .b(new_n213), .c(new_n214), .d(new_n205), .out0(new_n237));
  nor042aa1n06x5               g142(.a(new_n230), .b(new_n226), .o1(new_n238));
  and003aa1n02x5               g143(.a(new_n200), .b(new_n238), .c(new_n237), .o(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n190), .c(new_n120), .d(new_n182), .o1(new_n240));
  oaoi03aa1n12x5               g145(.a(\a[22] ), .b(\b[21] ), .c(new_n234), .o1(new_n241));
  aoi012aa1d24x5               g146(.a(new_n241), .b(new_n224), .c(new_n238), .o1(new_n242));
  xnrc02aa1n12x5               g147(.a(\b[22] ), .b(\a[23] ), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  xnbna2aa1n03x5               g149(.a(new_n244), .b(new_n240), .c(new_n242), .out0(\s[23] ));
  inv000aa1d42x5               g150(.a(new_n242), .o1(new_n246));
  aoai13aa1n03x5               g151(.a(new_n244), .b(new_n246), .c(new_n210), .d(new_n239), .o1(new_n247));
  tech160nm_fixorc02aa1n02p5x5 g152(.a(\a[24] ), .b(\b[23] ), .out0(new_n248));
  nor042aa1n03x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  norp02aa1n02x5               g154(.a(new_n248), .b(new_n249), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n249), .o1(new_n251));
  aoai13aa1n02x5               g156(.a(new_n251), .b(new_n243), .c(new_n240), .d(new_n242), .o1(new_n252));
  aoi022aa1n02x5               g157(.a(new_n252), .b(new_n248), .c(new_n247), .d(new_n250), .o1(\s[24] ));
  norb02aa1n03x5               g158(.a(new_n248), .b(new_n243), .out0(new_n254));
  inv000aa1n02x5               g159(.a(new_n254), .o1(new_n255));
  nano32aa1n02x4               g160(.a(new_n255), .b(new_n200), .c(new_n238), .d(new_n237), .out0(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n190), .c(new_n120), .d(new_n182), .o1(new_n257));
  inv000aa1n02x5               g162(.a(new_n223), .o1(new_n258));
  aoai13aa1n06x5               g163(.a(new_n238), .b(new_n258), .c(new_n237), .d(new_n211), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n241), .o1(new_n260));
  oao003aa1n02x5               g165(.a(\a[24] ), .b(\b[23] ), .c(new_n251), .carry(new_n261));
  aoai13aa1n12x5               g166(.a(new_n261), .b(new_n255), .c(new_n259), .d(new_n260), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  xorc02aa1n12x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  xnbna2aa1n03x5               g169(.a(new_n264), .b(new_n257), .c(new_n263), .out0(\s[25] ));
  aoai13aa1n03x5               g170(.a(new_n264), .b(new_n262), .c(new_n210), .d(new_n256), .o1(new_n266));
  xorc02aa1n02x5               g171(.a(\a[26] ), .b(\b[25] ), .out0(new_n267));
  nor042aa1n03x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  norp02aa1n02x5               g173(.a(new_n267), .b(new_n268), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n268), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n264), .o1(new_n271));
  aoai13aa1n02x5               g176(.a(new_n270), .b(new_n271), .c(new_n257), .d(new_n263), .o1(new_n272));
  aoi022aa1n03x5               g177(.a(new_n272), .b(new_n267), .c(new_n266), .d(new_n269), .o1(\s[26] ));
  and002aa1n06x5               g178(.a(new_n267), .b(new_n264), .o(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  nano32aa1n03x7               g180(.a(new_n275), .b(new_n221), .c(new_n238), .d(new_n254), .out0(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n190), .c(new_n120), .d(new_n182), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .c(new_n270), .carry(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  aoi012aa1n09x5               g184(.a(new_n279), .b(new_n262), .c(new_n274), .o1(new_n280));
  xorc02aa1n12x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n280), .c(new_n277), .out0(\s[27] ));
  aoai13aa1n04x5               g187(.a(new_n254), .b(new_n241), .c(new_n224), .d(new_n238), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n278), .b(new_n275), .c(new_n283), .d(new_n261), .o1(new_n284));
  aoai13aa1n02x5               g189(.a(new_n281), .b(new_n284), .c(new_n210), .d(new_n276), .o1(new_n285));
  xorc02aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .out0(new_n286));
  norp02aa1n02x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n286), .b(new_n287), .o1(new_n288));
  inv000aa1n03x5               g193(.a(new_n287), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n281), .o1(new_n290));
  aoai13aa1n03x5               g195(.a(new_n289), .b(new_n290), .c(new_n280), .d(new_n277), .o1(new_n291));
  aoi022aa1n03x5               g196(.a(new_n291), .b(new_n286), .c(new_n285), .d(new_n288), .o1(\s[28] ));
  and002aa1n02x5               g197(.a(new_n286), .b(new_n281), .o(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n284), .c(new_n210), .d(new_n276), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n293), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n289), .carry(new_n296));
  aoai13aa1n02x7               g201(.a(new_n296), .b(new_n295), .c(new_n280), .d(new_n277), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .out0(new_n298));
  norb02aa1n02x5               g203(.a(new_n296), .b(new_n298), .out0(new_n299));
  aoi022aa1n03x5               g204(.a(new_n297), .b(new_n298), .c(new_n294), .d(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1d33x5               g206(.a(new_n290), .b(new_n286), .c(new_n298), .out0(new_n302));
  aoai13aa1n02x5               g207(.a(new_n302), .b(new_n284), .c(new_n210), .d(new_n276), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n302), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n304), .c(new_n280), .d(new_n277), .o1(new_n306));
  xorc02aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .out0(new_n307));
  norb02aa1n02x5               g212(.a(new_n305), .b(new_n307), .out0(new_n308));
  aoi022aa1n03x5               g213(.a(new_n306), .b(new_n307), .c(new_n303), .d(new_n308), .o1(\s[30] ));
  nano32aa1d15x5               g214(.a(new_n290), .b(new_n307), .c(new_n286), .d(new_n298), .out0(new_n310));
  aoai13aa1n02x5               g215(.a(new_n310), .b(new_n284), .c(new_n210), .d(new_n276), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[31] ), .b(\b[30] ), .out0(new_n312));
  and002aa1n02x5               g217(.a(\b[29] ), .b(\a[30] ), .o(new_n313));
  oabi12aa1n02x5               g218(.a(new_n312), .b(\a[30] ), .c(\b[29] ), .out0(new_n314));
  oab012aa1n02x4               g219(.a(new_n314), .b(new_n305), .c(new_n313), .out0(new_n315));
  inv000aa1d42x5               g220(.a(new_n310), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n316), .c(new_n280), .d(new_n277), .o1(new_n318));
  aoi022aa1n03x5               g223(.a(new_n318), .b(new_n312), .c(new_n311), .d(new_n315), .o1(\s[31] ));
  xorb03aa1n02x5               g224(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi012aa1n02x5               g225(.a(new_n103), .b(\a[3] ), .c(\b[2] ), .o1(new_n321));
  oaib12aa1n02x5               g226(.a(new_n321), .b(new_n101), .c(new_n102), .out0(new_n322));
  oaib12aa1n02x5               g227(.a(new_n322), .b(new_n126), .c(new_n103), .out0(\s[4] ));
  xnbna2aa1n03x5               g228(.a(new_n109), .b(new_n104), .c(new_n106), .out0(\s[5] ));
  inv000aa1d42x5               g229(.a(\b[5] ), .o1(new_n325));
  nanp03aa1n02x5               g230(.a(new_n104), .b(new_n106), .c(new_n109), .o1(new_n326));
  aob012aa1n02x5               g231(.a(new_n326), .b(\b[4] ), .c(\a[5] ), .out0(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(new_n325), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g233(.a(\a[6] ), .b(\b[5] ), .c(new_n327), .carry(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[6] ), .c(new_n116), .out0(\s[7] ));
  aoai13aa1n02x5               g235(.a(new_n110), .b(new_n112), .c(new_n329), .d(new_n117), .o1(new_n331));
  norp02aa1n02x5               g236(.a(new_n110), .b(new_n112), .o1(new_n332));
  aob012aa1n02x5               g237(.a(new_n332), .b(new_n329), .c(new_n117), .out0(new_n333));
  nanp02aa1n02x5               g238(.a(new_n331), .b(new_n333), .o1(\s[8] ));
  xorb03aa1n02x5               g239(.a(new_n120), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


