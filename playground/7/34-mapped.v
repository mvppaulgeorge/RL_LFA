// Benchmark "adder" written by ABC on Wed Jul 17 15:49:14 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n319, new_n322, new_n324, new_n325, new_n327, new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n16x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oaoi03aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor002aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  oai012aa1n02x5               g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  tech160nm_fioai012aa1n05x5   g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n03x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nor002aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n03x5               g018(.a(new_n112), .b(new_n111), .c(new_n113), .d(new_n110), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n109), .b(new_n117), .o1(new_n118));
  nano23aa1n03x5               g023(.a(new_n113), .b(new_n110), .c(new_n111), .d(new_n112), .out0(new_n119));
  orn002aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .o(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n111), .b(new_n113), .c(new_n110), .o1(new_n122));
  aobi12aa1n06x5               g027(.a(new_n122), .b(new_n119), .c(new_n121), .out0(new_n123));
  nanp02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nanb02aa1n02x5               g029(.a(new_n97), .b(new_n124), .out0(new_n125));
  aoai13aa1n06x5               g030(.a(new_n98), .b(new_n125), .c(new_n118), .d(new_n123), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n12x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nand42aa1n04x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  nor042aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  and002aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o(new_n132));
  inv000aa1d42x5               g037(.a(new_n132), .o1(new_n133));
  aoai13aa1n04x5               g038(.a(new_n130), .b(new_n131), .c(new_n126), .d(new_n133), .o1(new_n134));
  oaib12aa1n02x5               g039(.a(new_n122), .b(new_n114), .c(new_n121), .out0(new_n135));
  inv000aa1d42x5               g040(.a(new_n125), .o1(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n135), .c(new_n109), .d(new_n117), .o1(new_n137));
  nor002aa1n03x5               g042(.a(new_n131), .b(new_n97), .o1(new_n138));
  aoi022aa1n02x5               g043(.a(new_n137), .b(new_n138), .c(\b[9] ), .d(\a[10] ), .o1(new_n139));
  oa0012aa1n02x5               g044(.a(new_n134), .b(new_n139), .c(new_n130), .o(\s[11] ));
  xorc02aa1n02x5               g045(.a(\a[12] ), .b(\b[11] ), .out0(new_n141));
  nona22aa1n02x4               g046(.a(new_n134), .b(new_n141), .c(new_n128), .out0(new_n142));
  inv040aa1n08x5               g047(.a(new_n128), .o1(new_n143));
  xnrc02aa1n12x5               g048(.a(\b[11] ), .b(\a[12] ), .out0(new_n144));
  aoi012aa1n02x5               g049(.a(new_n144), .b(new_n134), .c(new_n143), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n142), .b(new_n145), .out0(\s[12] ));
  norb03aa1n02x7               g051(.a(new_n124), .b(new_n97), .c(new_n131), .out0(new_n147));
  inv000aa1d42x5               g052(.a(\a[10] ), .o1(new_n148));
  inv000aa1d42x5               g053(.a(\b[9] ), .o1(new_n149));
  oai112aa1n06x5               g054(.a(new_n143), .b(new_n129), .c(new_n149), .d(new_n148), .o1(new_n150));
  nanb03aa1n06x5               g055(.a(new_n150), .b(new_n141), .c(new_n147), .out0(new_n151));
  oaoi03aa1n09x5               g056(.a(\a[12] ), .b(\b[11] ), .c(new_n143), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  oai013aa1d12x5               g058(.a(new_n153), .b(new_n150), .c(new_n144), .d(new_n138), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  aoai13aa1n06x5               g060(.a(new_n155), .b(new_n151), .c(new_n118), .d(new_n123), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n04x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n158), .b(new_n156), .c(new_n159), .o1(new_n160));
  xnrb03aa1n02x5               g065(.a(new_n160), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n03x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nand02aa1n03x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nona23aa1n09x5               g068(.a(new_n163), .b(new_n159), .c(new_n158), .d(new_n162), .out0(new_n164));
  inv000aa1n02x5               g069(.a(new_n164), .o1(new_n165));
  aoi012aa1n09x5               g070(.a(new_n162), .b(new_n158), .c(new_n163), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  nor022aa1n06x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand42aa1n03x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanb02aa1n09x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n167), .c(new_n156), .d(new_n165), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(new_n171), .b(new_n167), .c(new_n156), .d(new_n165), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(\s[15] ));
  nor042aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanb02aa1n02x5               g081(.a(new_n175), .b(new_n176), .out0(new_n177));
  oai112aa1n02x5               g082(.a(new_n172), .b(new_n177), .c(\b[14] ), .d(\a[15] ), .o1(new_n178));
  oaoi13aa1n02x5               g083(.a(new_n177), .b(new_n172), .c(\a[15] ), .d(\b[14] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n178), .b(new_n179), .out0(\s[16] ));
  nano23aa1n02x4               g085(.a(new_n168), .b(new_n175), .c(new_n176), .d(new_n169), .out0(new_n181));
  nano22aa1n03x7               g086(.a(new_n151), .b(new_n165), .c(new_n181), .out0(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n135), .c(new_n109), .d(new_n117), .o1(new_n183));
  nor043aa1n02x5               g088(.a(new_n164), .b(new_n170), .c(new_n177), .o1(new_n184));
  nor003aa1n03x5               g089(.a(new_n166), .b(new_n170), .c(new_n177), .o1(new_n185));
  oa0012aa1n02x5               g090(.a(new_n176), .b(new_n175), .c(new_n168), .o(new_n186));
  aoi112aa1n09x5               g091(.a(new_n186), .b(new_n185), .c(new_n154), .d(new_n184), .o1(new_n187));
  nand22aa1n03x5               g092(.a(new_n183), .b(new_n187), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g094(.a(\a[18] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\a[17] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\b[16] ), .o1(new_n192));
  oaoi03aa1n03x5               g097(.a(new_n191), .b(new_n192), .c(new_n188), .o1(new_n193));
  xorb03aa1n02x5               g098(.a(new_n193), .b(\b[17] ), .c(new_n190), .out0(\s[18] ));
  xroi22aa1d06x4               g099(.a(new_n191), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n195));
  inv000aa1d42x5               g100(.a(new_n195), .o1(new_n196));
  norp02aa1n02x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  aoi112aa1n03x5               g102(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n198));
  norp02aa1n04x5               g103(.a(new_n198), .b(new_n197), .o1(new_n199));
  aoai13aa1n04x5               g104(.a(new_n199), .b(new_n196), .c(new_n183), .d(new_n187), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n08x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nand02aa1n04x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  norb02aa1n12x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  nor022aa1n06x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nand42aa1n08x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n03x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  aoi112aa1n02x5               g113(.a(new_n203), .b(new_n208), .c(new_n200), .d(new_n205), .o1(new_n209));
  aoai13aa1n03x5               g114(.a(new_n208), .b(new_n203), .c(new_n200), .d(new_n204), .o1(new_n210));
  norb02aa1n02x7               g115(.a(new_n210), .b(new_n209), .out0(\s[20] ));
  nona23aa1d18x5               g116(.a(new_n207), .b(new_n204), .c(new_n203), .d(new_n206), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(new_n195), .b(new_n213), .o1(new_n214));
  tech160nm_fioai012aa1n05x5   g119(.a(new_n207), .b(new_n206), .c(new_n203), .o1(new_n215));
  oai012aa1n12x5               g120(.a(new_n215), .b(new_n212), .c(new_n199), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  aoai13aa1n04x5               g122(.a(new_n217), .b(new_n214), .c(new_n183), .d(new_n187), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  xorc02aa1n12x5               g125(.a(\a[21] ), .b(\b[20] ), .out0(new_n221));
  xorc02aa1n12x5               g126(.a(\a[22] ), .b(\b[21] ), .out0(new_n222));
  aoi112aa1n02x5               g127(.a(new_n220), .b(new_n222), .c(new_n218), .d(new_n221), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n222), .b(new_n220), .c(new_n218), .d(new_n221), .o1(new_n224));
  norb02aa1n02x7               g129(.a(new_n224), .b(new_n223), .out0(\s[22] ));
  nand02aa1d06x5               g130(.a(new_n222), .b(new_n221), .o1(new_n226));
  nanb03aa1n09x5               g131(.a(new_n226), .b(new_n195), .c(new_n213), .out0(new_n227));
  oai112aa1n04x5               g132(.a(new_n205), .b(new_n208), .c(new_n198), .d(new_n197), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\a[22] ), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\b[21] ), .o1(new_n230));
  oaoi03aa1n09x5               g135(.a(new_n229), .b(new_n230), .c(new_n220), .o1(new_n231));
  aoai13aa1n12x5               g136(.a(new_n231), .b(new_n226), .c(new_n228), .d(new_n215), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n04x5               g138(.a(new_n233), .b(new_n227), .c(new_n183), .d(new_n187), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  xorc02aa1n02x5               g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  xorc02aa1n02x5               g142(.a(\a[24] ), .b(\b[23] ), .out0(new_n238));
  aoi112aa1n02x5               g143(.a(new_n236), .b(new_n238), .c(new_n234), .d(new_n237), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n238), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n240));
  norb02aa1n02x7               g145(.a(new_n240), .b(new_n239), .out0(\s[24] ));
  and002aa1n02x5               g146(.a(new_n238), .b(new_n237), .o(new_n242));
  nona23aa1n02x4               g147(.a(new_n242), .b(new_n195), .c(new_n226), .d(new_n212), .out0(new_n243));
  inv000aa1d42x5               g148(.a(\a[24] ), .o1(new_n244));
  inv000aa1d42x5               g149(.a(\b[23] ), .o1(new_n245));
  oao003aa1n02x5               g150(.a(new_n244), .b(new_n245), .c(new_n236), .carry(new_n246));
  tech160nm_fiaoi012aa1n05x5   g151(.a(new_n246), .b(new_n232), .c(new_n242), .o1(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n243), .c(new_n183), .d(new_n187), .o1(new_n248));
  xorb03aa1n02x5               g153(.a(new_n248), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g154(.a(\b[24] ), .b(\a[25] ), .o1(new_n250));
  tech160nm_fixorc02aa1n03p5x5 g155(.a(\a[25] ), .b(\b[24] ), .out0(new_n251));
  tech160nm_fixorc02aa1n02p5x5 g156(.a(\a[26] ), .b(\b[25] ), .out0(new_n252));
  aoi112aa1n03x5               g157(.a(new_n250), .b(new_n252), .c(new_n248), .d(new_n251), .o1(new_n253));
  aoai13aa1n03x5               g158(.a(new_n252), .b(new_n250), .c(new_n248), .d(new_n251), .o1(new_n254));
  norb02aa1n03x4               g159(.a(new_n254), .b(new_n253), .out0(\s[26] ));
  oao003aa1n02x5               g160(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n256));
  nano23aa1n02x4               g161(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n257));
  aobi12aa1n02x5               g162(.a(new_n108), .b(new_n257), .c(new_n256), .out0(new_n258));
  nona22aa1n02x4               g163(.a(new_n119), .b(new_n115), .c(new_n116), .out0(new_n259));
  oai012aa1n02x5               g164(.a(new_n123), .b(new_n258), .c(new_n259), .o1(new_n260));
  norp03aa1n02x5               g165(.a(new_n150), .b(new_n144), .c(new_n138), .o1(new_n261));
  oai012aa1n02x5               g166(.a(new_n184), .b(new_n261), .c(new_n152), .o1(new_n262));
  nona22aa1n02x4               g167(.a(new_n262), .b(new_n185), .c(new_n186), .out0(new_n263));
  and002aa1n06x5               g168(.a(new_n252), .b(new_n251), .o(new_n264));
  nano22aa1n03x7               g169(.a(new_n227), .b(new_n242), .c(new_n264), .out0(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n263), .c(new_n260), .d(new_n182), .o1(new_n266));
  aoai13aa1n09x5               g171(.a(new_n264), .b(new_n246), .c(new_n232), .d(new_n242), .o1(new_n267));
  oai022aa1n02x5               g172(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n268));
  aob012aa1n02x5               g173(.a(new_n268), .b(\b[25] ), .c(\a[26] ), .out0(new_n269));
  xorc02aa1n12x5               g174(.a(\a[27] ), .b(\b[26] ), .out0(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aoi013aa1n03x5               g176(.a(new_n271), .b(new_n266), .c(new_n267), .d(new_n269), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n226), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n231), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n242), .b(new_n274), .c(new_n216), .d(new_n273), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n246), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n264), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n269), .b(new_n277), .c(new_n275), .d(new_n276), .o1(new_n278));
  aoi112aa1n02x5               g183(.a(new_n278), .b(new_n270), .c(new_n188), .d(new_n265), .o1(new_n279));
  norp02aa1n02x5               g184(.a(new_n272), .b(new_n279), .o1(\s[27] ));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  inv040aa1n03x5               g186(.a(new_n281), .o1(new_n282));
  xnrc02aa1n12x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  nano22aa1n03x5               g188(.a(new_n272), .b(new_n282), .c(new_n283), .out0(new_n284));
  inv020aa1n02x5               g189(.a(new_n265), .o1(new_n285));
  aoi012aa1n06x5               g190(.a(new_n285), .b(new_n183), .c(new_n187), .o1(new_n286));
  oaih12aa1n02x5               g191(.a(new_n270), .b(new_n278), .c(new_n286), .o1(new_n287));
  tech160nm_fiaoi012aa1n02p5x5 g192(.a(new_n283), .b(new_n287), .c(new_n282), .o1(new_n288));
  norp02aa1n03x5               g193(.a(new_n288), .b(new_n284), .o1(\s[28] ));
  norb02aa1d21x5               g194(.a(new_n270), .b(new_n283), .out0(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  aoi013aa1n02x5               g196(.a(new_n291), .b(new_n266), .c(new_n267), .d(new_n269), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  nano22aa1n03x5               g199(.a(new_n292), .b(new_n293), .c(new_n294), .out0(new_n295));
  oaih12aa1n02x5               g200(.a(new_n290), .b(new_n278), .c(new_n286), .o1(new_n296));
  tech160nm_fiaoi012aa1n02p5x5 g201(.a(new_n294), .b(new_n296), .c(new_n293), .o1(new_n297));
  norp02aa1n03x5               g202(.a(new_n297), .b(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g204(.a(new_n270), .b(new_n294), .c(new_n283), .out0(new_n300));
  oaih12aa1n02x5               g205(.a(new_n300), .b(new_n278), .c(new_n286), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n303), .b(new_n301), .c(new_n302), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n300), .o1(new_n305));
  aoi013aa1n03x5               g210(.a(new_n305), .b(new_n266), .c(new_n267), .d(new_n269), .o1(new_n306));
  nano22aa1n03x5               g211(.a(new_n306), .b(new_n302), .c(new_n303), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n304), .b(new_n307), .o1(\s[30] ));
  norb02aa1n02x5               g213(.a(new_n300), .b(new_n303), .out0(new_n309));
  inv000aa1n02x5               g214(.a(new_n309), .o1(new_n310));
  aoi013aa1n02x5               g215(.a(new_n310), .b(new_n266), .c(new_n267), .d(new_n269), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[30] ), .b(\a[31] ), .out0(new_n313));
  nano22aa1n03x5               g218(.a(new_n311), .b(new_n312), .c(new_n313), .out0(new_n314));
  oaih12aa1n02x5               g219(.a(new_n309), .b(new_n278), .c(new_n286), .o1(new_n315));
  tech160nm_fiaoi012aa1n02p5x5 g220(.a(new_n313), .b(new_n315), .c(new_n312), .o1(new_n316));
  norp02aa1n03x5               g221(.a(new_n316), .b(new_n314), .o1(\s[31] ));
  xnrb03aa1n02x5               g222(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g223(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g225(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g226(.a(\a[5] ), .b(\b[4] ), .c(new_n258), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g228(.a(\a[5] ), .b(\b[4] ), .c(new_n258), .carry(new_n324));
  oaoi03aa1n02x5               g229(.a(\a[6] ), .b(\b[5] ), .c(new_n324), .o1(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  inv000aa1d42x5               g231(.a(\a[8] ), .o1(new_n327));
  aoi012aa1n02x5               g232(.a(new_n113), .b(new_n325), .c(new_n112), .o1(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[7] ), .c(new_n327), .out0(\s[8] ));
  xnbna2aa1n03x5               g234(.a(new_n136), .b(new_n118), .c(new_n123), .out0(\s[9] ));
endmodule


