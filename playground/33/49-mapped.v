// Benchmark "adder" written by ABC on Thu Jul 18 05:19:46 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n130, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n146, new_n147, new_n148, new_n150,
    new_n151, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n161, new_n162, new_n163, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n174,
    new_n175, new_n176, new_n177, new_n179, new_n180, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n198, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n206, new_n207,
    new_n208, new_n209, new_n210, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n224, new_n225, new_n226, new_n227, new_n228, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n240, new_n241, new_n242, new_n243, new_n245, new_n246, new_n247,
    new_n248, new_n249, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n305, new_n306, new_n307, new_n309, new_n310, new_n311,
    new_n312, new_n315, new_n316, new_n318, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n03x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand42aa1n16x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nanp02aa1n24x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1n09x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  tech160nm_fixnrc02aa1n05x5   g005(.a(\b[2] ), .b(\a[3] ), .out0(new_n101));
  inv000aa1d42x5               g006(.a(\a[3] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\a[4] ), .o1(new_n103));
  inv000aa1d42x5               g008(.a(\b[2] ), .o1(new_n104));
  aboi22aa1d24x5               g009(.a(\b[3] ), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n105));
  oai012aa1n09x5               g010(.a(new_n105), .b(new_n101), .c(new_n100), .o1(new_n106));
  nand42aa1n06x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nor002aa1n03x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  norb02aa1n06x4               g013(.a(new_n107), .b(new_n108), .out0(new_n109));
  aoi022aa1d24x5               g014(.a(\b[6] ), .b(\a[7] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n110));
  nor002aa1n16x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  inv040aa1n03x5               g016(.a(new_n111), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nanp02aa1n06x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\b[7] ), .o1(new_n115));
  nanb02aa1d24x5               g020(.a(\a[8] ), .b(new_n115), .out0(new_n116));
  oai112aa1n06x5               g021(.a(new_n116), .b(new_n114), .c(\b[6] ), .d(\a[7] ), .o1(new_n117));
  nano32aa1n03x7               g022(.a(new_n117), .b(new_n110), .c(new_n112), .d(new_n113), .out0(new_n118));
  nanp02aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  aoai13aa1n12x5               g024(.a(new_n119), .b(new_n108), .c(new_n111), .d(new_n107), .o1(new_n120));
  aboi22aa1n12x5               g025(.a(new_n117), .b(new_n120), .c(\a[8] ), .d(\b[7] ), .out0(new_n121));
  aoi013aa1n09x5               g026(.a(new_n121), .b(new_n106), .c(new_n118), .d(new_n109), .o1(new_n122));
  oaoi03aa1n09x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  inv000aa1d42x5               g029(.a(\a[11] ), .o1(new_n125));
  nor002aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand42aa1n06x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  oai012aa1n04x7               g032(.a(new_n127), .b(new_n123), .c(new_n126), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[10] ), .c(new_n125), .out0(\s[11] ));
  inv000aa1d42x5               g034(.a(\b[10] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(new_n130), .b(new_n125), .o1(new_n131));
  xnrc02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .out0(new_n132));
  xnrc02aa1n02x5               g037(.a(\b[11] ), .b(\a[12] ), .out0(new_n133));
  oaoi13aa1n02x5               g038(.a(new_n133), .b(new_n131), .c(new_n128), .d(new_n132), .o1(new_n134));
  oai112aa1n02x5               g039(.a(new_n131), .b(new_n133), .c(new_n128), .d(new_n132), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(\s[12] ));
  norb02aa1n02x5               g041(.a(new_n127), .b(new_n126), .out0(new_n137));
  xorc02aa1n12x5               g042(.a(\a[9] ), .b(\b[8] ), .out0(new_n138));
  nona23aa1n02x4               g043(.a(new_n137), .b(new_n138), .c(new_n133), .d(new_n132), .out0(new_n139));
  oai022aa1d18x5               g044(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n140));
  oai112aa1n03x5               g045(.a(new_n140), .b(new_n127), .c(new_n130), .d(new_n125), .o1(new_n141));
  oa0022aa1n02x5               g046(.a(\a[12] ), .b(\b[11] ), .c(\a[11] ), .d(\b[10] ), .o(new_n142));
  aoi022aa1n02x7               g047(.a(new_n141), .b(new_n142), .c(\b[11] ), .d(\a[12] ), .o1(new_n143));
  oabi12aa1n06x5               g048(.a(new_n143), .b(new_n122), .c(new_n139), .out0(new_n144));
  xorb03aa1n02x5               g049(.a(new_n144), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n16x5               g050(.a(\b[12] ), .b(\a[13] ), .o1(new_n146));
  nand02aa1n04x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  aoi012aa1n02x5               g052(.a(new_n146), .b(new_n144), .c(new_n147), .o1(new_n148));
  xnrb03aa1n02x5               g053(.a(new_n148), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n16x5               g054(.a(\b[13] ), .b(\a[14] ), .o1(new_n150));
  nand22aa1n09x5               g055(.a(\b[13] ), .b(\a[14] ), .o1(new_n151));
  nona23aa1d16x5               g056(.a(new_n151), .b(new_n147), .c(new_n146), .d(new_n150), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  aoi012aa1n12x5               g058(.a(new_n150), .b(new_n146), .c(new_n151), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  xnrc02aa1n12x5               g060(.a(\b[14] ), .b(\a[15] ), .out0(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  aoai13aa1n06x5               g062(.a(new_n157), .b(new_n155), .c(new_n144), .d(new_n153), .o1(new_n158));
  aoi112aa1n02x5               g063(.a(new_n157), .b(new_n155), .c(new_n144), .d(new_n153), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n158), .b(new_n159), .out0(\s[15] ));
  xnrc02aa1n12x5               g065(.a(\b[15] ), .b(\a[16] ), .out0(new_n161));
  oaoi13aa1n06x5               g066(.a(new_n161), .b(new_n158), .c(\a[15] ), .d(\b[14] ), .o1(new_n162));
  oai112aa1n03x5               g067(.a(new_n158), .b(new_n161), .c(\b[14] ), .d(\a[15] ), .o1(new_n163));
  norb02aa1n03x4               g068(.a(new_n163), .b(new_n162), .out0(\s[16] ));
  nanp02aa1n02x5               g069(.a(new_n138), .b(new_n137), .o1(new_n165));
  nor043aa1d12x5               g070(.a(new_n152), .b(new_n156), .c(new_n161), .o1(new_n166));
  nona32aa1n09x5               g071(.a(new_n166), .b(new_n165), .c(new_n133), .d(new_n132), .out0(new_n167));
  norp02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  ao0022aa1n03x5               g073(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o(new_n169));
  oaoi13aa1n04x5               g074(.a(new_n169), .b(new_n154), .c(\a[15] ), .d(\b[14] ), .o1(new_n170));
  aoi112aa1n06x5               g075(.a(new_n170), .b(new_n168), .c(new_n166), .d(new_n143), .o1(new_n171));
  oai012aa1n18x5               g076(.a(new_n171), .b(new_n122), .c(new_n167), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g078(.a(\a[18] ), .o1(new_n174));
  inv040aa1d30x5               g079(.a(\a[17] ), .o1(new_n175));
  inv000aa1d42x5               g080(.a(\b[16] ), .o1(new_n176));
  oaoi03aa1n02x5               g081(.a(new_n175), .b(new_n176), .c(new_n172), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[17] ), .c(new_n174), .out0(\s[18] ));
  xroi22aa1d06x4               g083(.a(new_n175), .b(\b[16] ), .c(new_n174), .d(\b[17] ), .out0(new_n179));
  nor042aa1n02x5               g084(.a(\b[17] ), .b(\a[18] ), .o1(new_n180));
  aoi112aa1n09x5               g085(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n181));
  nor042aa1n06x5               g086(.a(new_n181), .b(new_n180), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  norp02aa1n09x5               g088(.a(\b[18] ), .b(\a[19] ), .o1(new_n184));
  nand22aa1n03x5               g089(.a(\b[18] ), .b(\a[19] ), .o1(new_n185));
  norb02aa1n03x5               g090(.a(new_n185), .b(new_n184), .out0(new_n186));
  aoai13aa1n06x5               g091(.a(new_n186), .b(new_n183), .c(new_n172), .d(new_n179), .o1(new_n187));
  aoi112aa1n02x5               g092(.a(new_n186), .b(new_n183), .c(new_n172), .d(new_n179), .o1(new_n188));
  norb02aa1n02x5               g093(.a(new_n187), .b(new_n188), .out0(\s[19] ));
  xnrc02aa1n02x5               g094(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n24x5               g095(.a(\b[19] ), .b(\a[20] ), .o1(new_n191));
  nand02aa1d16x5               g096(.a(\b[19] ), .b(\a[20] ), .o1(new_n192));
  norb02aa1d21x5               g097(.a(new_n192), .b(new_n191), .out0(new_n193));
  nona22aa1n03x5               g098(.a(new_n187), .b(new_n193), .c(new_n184), .out0(new_n194));
  inv000aa1d42x5               g099(.a(new_n193), .o1(new_n195));
  oaoi13aa1n06x5               g100(.a(new_n195), .b(new_n187), .c(\a[19] ), .d(\b[18] ), .o1(new_n196));
  norb02aa1n03x4               g101(.a(new_n194), .b(new_n196), .out0(\s[20] ));
  nona23aa1n08x5               g102(.a(new_n192), .b(new_n185), .c(new_n184), .d(new_n191), .out0(new_n198));
  norb02aa1n06x5               g103(.a(new_n179), .b(new_n198), .out0(new_n199));
  oai012aa1n02x5               g104(.a(new_n192), .b(new_n191), .c(new_n184), .o1(new_n200));
  tech160nm_fioai012aa1n03p5x5 g105(.a(new_n200), .b(new_n198), .c(new_n182), .o1(new_n201));
  xorc02aa1n12x5               g106(.a(\a[21] ), .b(\b[20] ), .out0(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n201), .c(new_n172), .d(new_n199), .o1(new_n203));
  aoi112aa1n02x5               g108(.a(new_n202), .b(new_n201), .c(new_n172), .d(new_n199), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(\s[21] ));
  nor042aa1n03x5               g110(.a(\b[20] ), .b(\a[21] ), .o1(new_n206));
  xorc02aa1n12x5               g111(.a(\a[22] ), .b(\b[21] ), .out0(new_n207));
  nona22aa1n03x5               g112(.a(new_n203), .b(new_n207), .c(new_n206), .out0(new_n208));
  inv040aa1n03x5               g113(.a(new_n206), .o1(new_n209));
  aobi12aa1n06x5               g114(.a(new_n207), .b(new_n203), .c(new_n209), .out0(new_n210));
  norb02aa1n03x4               g115(.a(new_n208), .b(new_n210), .out0(\s[22] ));
  inv040aa1n03x5               g116(.a(new_n198), .o1(new_n212));
  nand02aa1n08x5               g117(.a(new_n207), .b(new_n202), .o1(new_n213));
  nanb03aa1d18x5               g118(.a(new_n213), .b(new_n179), .c(new_n212), .out0(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  oai112aa1n04x5               g120(.a(new_n186), .b(new_n193), .c(new_n181), .d(new_n180), .o1(new_n216));
  oaoi03aa1n09x5               g121(.a(\a[22] ), .b(\b[21] ), .c(new_n209), .o1(new_n217));
  inv040aa1d30x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n213), .c(new_n216), .d(new_n200), .o1(new_n219));
  xorc02aa1n03x5               g124(.a(\a[23] ), .b(\b[22] ), .out0(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n219), .c(new_n172), .d(new_n215), .o1(new_n221));
  aoi112aa1n02x5               g126(.a(new_n220), .b(new_n219), .c(new_n172), .d(new_n215), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n221), .b(new_n222), .out0(\s[23] ));
  nor042aa1n04x5               g128(.a(\b[22] ), .b(\a[23] ), .o1(new_n224));
  xorc02aa1n02x5               g129(.a(\a[24] ), .b(\b[23] ), .out0(new_n225));
  nona22aa1n03x5               g130(.a(new_n221), .b(new_n225), .c(new_n224), .out0(new_n226));
  inv020aa1n02x5               g131(.a(new_n224), .o1(new_n227));
  aobi12aa1n06x5               g132(.a(new_n225), .b(new_n221), .c(new_n227), .out0(new_n228));
  norb02aa1n03x4               g133(.a(new_n226), .b(new_n228), .out0(\s[24] ));
  nanp02aa1n02x5               g134(.a(new_n225), .b(new_n220), .o1(new_n230));
  nor042aa1n02x5               g135(.a(new_n214), .b(new_n230), .o1(new_n231));
  inv000aa1n02x5               g136(.a(new_n230), .o1(new_n232));
  oaoi03aa1n12x5               g137(.a(\a[24] ), .b(\b[23] ), .c(new_n227), .o1(new_n233));
  aoi012aa1n02x5               g138(.a(new_n233), .b(new_n219), .c(new_n232), .o1(new_n234));
  inv020aa1n02x5               g139(.a(new_n234), .o1(new_n235));
  tech160nm_fixorc02aa1n05x5   g140(.a(\a[25] ), .b(\b[24] ), .out0(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n235), .c(new_n172), .d(new_n231), .o1(new_n237));
  aoi112aa1n02x5               g142(.a(new_n235), .b(new_n236), .c(new_n172), .d(new_n231), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n237), .b(new_n238), .out0(\s[25] ));
  xorc02aa1n12x5               g144(.a(\a[26] ), .b(\b[25] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  oai112aa1n03x5               g146(.a(new_n237), .b(new_n241), .c(\b[24] ), .d(\a[25] ), .o1(new_n242));
  oaoi13aa1n06x5               g147(.a(new_n241), .b(new_n237), .c(\a[25] ), .d(\b[24] ), .o1(new_n243));
  norb02aa1n03x4               g148(.a(new_n242), .b(new_n243), .out0(\s[26] ));
  nanp03aa1n02x5               g149(.a(new_n106), .b(new_n109), .c(new_n118), .o1(new_n245));
  nanb02aa1n02x5               g150(.a(new_n121), .b(new_n245), .out0(new_n246));
  norb02aa1n03x5               g151(.a(new_n166), .b(new_n139), .out0(new_n247));
  nanp02aa1n02x5               g152(.a(new_n166), .b(new_n143), .o1(new_n248));
  nona22aa1n02x4               g153(.a(new_n248), .b(new_n170), .c(new_n168), .out0(new_n249));
  nand02aa1n02x5               g154(.a(new_n240), .b(new_n236), .o1(new_n250));
  inv000aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  nano22aa1n03x7               g156(.a(new_n214), .b(new_n232), .c(new_n251), .out0(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n249), .c(new_n246), .d(new_n247), .o1(new_n253));
  aoai13aa1n04x5               g158(.a(new_n251), .b(new_n233), .c(new_n219), .d(new_n232), .o1(new_n254));
  oai022aa1n02x5               g159(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n255));
  aob012aa1n02x5               g160(.a(new_n255), .b(\b[25] ), .c(\a[26] ), .out0(new_n256));
  norp02aa1n02x5               g161(.a(\b[26] ), .b(\a[27] ), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(\b[26] ), .b(\a[27] ), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n258), .b(new_n257), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  aoi013aa1n03x5               g165(.a(new_n260), .b(new_n253), .c(new_n254), .d(new_n256), .o1(new_n261));
  nona32aa1n02x4               g166(.a(new_n199), .b(new_n250), .c(new_n230), .d(new_n213), .out0(new_n262));
  oaoi13aa1n06x5               g167(.a(new_n262), .b(new_n171), .c(new_n122), .d(new_n167), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n213), .o1(new_n264));
  aoai13aa1n04x5               g169(.a(new_n232), .b(new_n217), .c(new_n201), .d(new_n264), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n233), .o1(new_n266));
  aoai13aa1n06x5               g171(.a(new_n256), .b(new_n250), .c(new_n265), .d(new_n266), .o1(new_n267));
  norp03aa1n02x5               g172(.a(new_n267), .b(new_n263), .c(new_n259), .o1(new_n268));
  norp02aa1n02x5               g173(.a(new_n261), .b(new_n268), .o1(\s[27] ));
  inv020aa1n02x5               g174(.a(new_n257), .o1(new_n270));
  xnrc02aa1n12x5               g175(.a(\b[27] ), .b(\a[28] ), .out0(new_n271));
  nano22aa1n03x5               g176(.a(new_n261), .b(new_n270), .c(new_n271), .out0(new_n272));
  oaih12aa1n02x5               g177(.a(new_n259), .b(new_n267), .c(new_n263), .o1(new_n273));
  tech160nm_fiaoi012aa1n02p5x5 g178(.a(new_n271), .b(new_n273), .c(new_n270), .o1(new_n274));
  norp02aa1n03x5               g179(.a(new_n274), .b(new_n272), .o1(\s[28] ));
  nano22aa1n09x5               g180(.a(new_n271), .b(new_n270), .c(new_n258), .out0(new_n276));
  oaih12aa1n02x5               g181(.a(new_n276), .b(new_n267), .c(new_n263), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[28] ), .b(\b[27] ), .c(new_n270), .carry(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[28] ), .b(\a[29] ), .out0(new_n279));
  tech160nm_fiaoi012aa1n02p5x5 g184(.a(new_n279), .b(new_n277), .c(new_n278), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n276), .o1(new_n281));
  aoi013aa1n03x5               g186(.a(new_n281), .b(new_n253), .c(new_n254), .d(new_n256), .o1(new_n282));
  nano22aa1n03x5               g187(.a(new_n282), .b(new_n278), .c(new_n279), .out0(new_n283));
  norp02aa1n03x5               g188(.a(new_n280), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g189(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g190(.a(new_n279), .b(new_n271), .c(new_n258), .d(new_n270), .out0(new_n286));
  oaih12aa1n02x5               g191(.a(new_n286), .b(new_n267), .c(new_n263), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[29] ), .b(\b[28] ), .c(new_n278), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[29] ), .b(\a[30] ), .out0(new_n289));
  tech160nm_fiaoi012aa1n02p5x5 g194(.a(new_n289), .b(new_n287), .c(new_n288), .o1(new_n290));
  inv000aa1n02x5               g195(.a(new_n286), .o1(new_n291));
  aoi013aa1n03x5               g196(.a(new_n291), .b(new_n253), .c(new_n254), .d(new_n256), .o1(new_n292));
  nano22aa1n03x5               g197(.a(new_n292), .b(new_n288), .c(new_n289), .out0(new_n293));
  norp02aa1n03x5               g198(.a(new_n290), .b(new_n293), .o1(\s[30] ));
  norb03aa1n02x5               g199(.a(new_n276), .b(new_n289), .c(new_n279), .out0(new_n295));
  oaih12aa1n02x5               g200(.a(new_n295), .b(new_n267), .c(new_n263), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[30] ), .b(\b[29] ), .c(new_n288), .carry(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[30] ), .b(\a[31] ), .out0(new_n298));
  tech160nm_fiaoi012aa1n02p5x5 g203(.a(new_n298), .b(new_n296), .c(new_n297), .o1(new_n299));
  inv000aa1n02x5               g204(.a(new_n295), .o1(new_n300));
  aoi013aa1n03x5               g205(.a(new_n300), .b(new_n253), .c(new_n254), .d(new_n256), .o1(new_n301));
  nano22aa1n03x5               g206(.a(new_n301), .b(new_n297), .c(new_n298), .out0(new_n302));
  norp02aa1n03x5               g207(.a(new_n299), .b(new_n302), .o1(\s[31] ));
  xorb03aa1n02x5               g208(.a(new_n100), .b(\b[2] ), .c(new_n102), .out0(\s[3] ));
  nanp02aa1n02x5               g209(.a(new_n104), .b(new_n102), .o1(new_n305));
  xorc02aa1n02x5               g210(.a(\a[4] ), .b(\b[3] ), .out0(new_n306));
  oab012aa1n02x4               g211(.a(new_n306), .b(new_n101), .c(new_n100), .out0(new_n307));
  aoi022aa1n02x5               g212(.a(new_n307), .b(new_n305), .c(new_n106), .d(new_n306), .o1(\s[4] ));
  inv000aa1d42x5               g213(.a(\b[3] ), .o1(new_n309));
  norb02aa1n02x5               g214(.a(new_n113), .b(new_n111), .out0(new_n310));
  oaoi13aa1n02x5               g215(.a(new_n310), .b(new_n106), .c(new_n103), .d(new_n309), .o1(new_n311));
  oai112aa1n02x5               g216(.a(new_n106), .b(new_n310), .c(new_n309), .d(new_n103), .o1(new_n312));
  norb02aa1n02x5               g217(.a(new_n312), .b(new_n311), .out0(\s[5] ));
  xnbna2aa1n03x5               g218(.a(new_n109), .b(new_n312), .c(new_n112), .out0(\s[6] ));
  nanp03aa1n02x5               g219(.a(new_n312), .b(new_n109), .c(new_n112), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[7] ), .b(\b[6] ), .out0(new_n316));
  xobna2aa1n03x5               g221(.a(new_n316), .b(new_n315), .c(new_n107), .out0(\s[7] ));
  nanp03aa1n02x5               g222(.a(new_n315), .b(new_n107), .c(new_n316), .o1(new_n318));
  oai012aa1n02x5               g223(.a(new_n318), .b(\b[6] ), .c(\a[7] ), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrc02aa1n02x5               g225(.a(new_n122), .b(new_n138), .out0(\s[9] ));
endmodule


