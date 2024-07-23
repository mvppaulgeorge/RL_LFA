// Benchmark "adder" written by ABC on Wed Jul 17 16:44:57 2024

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
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n323, new_n324,
    new_n327, new_n329, new_n331, new_n332, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv040aa1d32x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nand02aa1d04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  tech160nm_fioaoi03aa1n02p5x5 g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor022aa1n12x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n16x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n03x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  inv000aa1d42x5               g012(.a(\a[3] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\b[2] ), .o1(new_n109));
  aoai13aa1n02x7               g014(.a(new_n104), .b(new_n103), .c(new_n108), .d(new_n109), .o1(new_n110));
  oaih12aa1n06x5               g015(.a(new_n110), .b(new_n107), .c(new_n102), .o1(new_n111));
  nand02aa1n04x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor022aa1n16x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nor042aa1n06x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nona23aa1n03x5               g020(.a(new_n112), .b(new_n115), .c(new_n114), .d(new_n113), .out0(new_n116));
  nor022aa1n08x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand42aa1d28x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nanb02aa1n02x5               g023(.a(new_n117), .b(new_n118), .out0(new_n119));
  nor042aa1n06x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nand42aa1n06x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  nanb02aa1n02x5               g026(.a(new_n120), .b(new_n121), .out0(new_n122));
  nor043aa1n06x5               g027(.a(new_n116), .b(new_n119), .c(new_n122), .o1(new_n123));
  nano23aa1n03x7               g028(.a(new_n117), .b(new_n120), .c(new_n121), .d(new_n118), .out0(new_n124));
  tech160nm_fiao0012aa1n02p5x5 g029(.a(new_n113), .b(new_n114), .c(new_n112), .o(new_n125));
  aoi012aa1n02x5               g030(.a(new_n117), .b(new_n120), .c(new_n118), .o1(new_n126));
  aob012aa1n06x5               g031(.a(new_n126), .b(new_n124), .c(new_n125), .out0(new_n127));
  xorc02aa1n06x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n127), .c(new_n111), .d(new_n123), .o1(new_n129));
  tech160nm_fixorc02aa1n03p5x5 g034(.a(\a[10] ), .b(\b[9] ), .out0(new_n130));
  nand02aa1n06x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  oaih22aa1n04x5               g037(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n133));
  nona22aa1n02x4               g038(.a(new_n129), .b(new_n132), .c(new_n133), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n130), .c(new_n98), .d(new_n129), .o1(\s[10] ));
  nanb02aa1n06x5               g040(.a(new_n133), .b(new_n129), .out0(new_n136));
  nand42aa1d28x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nor042aa1n06x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nona23aa1n06x5               g043(.a(new_n136), .b(new_n137), .c(new_n138), .d(new_n132), .out0(new_n139));
  aboi22aa1n03x5               g044(.a(new_n138), .b(new_n137), .c(new_n136), .d(new_n131), .out0(new_n140));
  norb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(\s[11] ));
  orn002aa1n12x5               g046(.a(\a[11] ), .b(\b[10] ), .o(new_n142));
  norp02aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand42aa1n20x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n139), .c(new_n142), .out0(\s[12] ));
  nanp02aa1n02x5               g051(.a(new_n111), .b(new_n123), .o1(new_n147));
  aobi12aa1n06x5               g052(.a(new_n126), .b(new_n124), .c(new_n125), .out0(new_n148));
  nano23aa1n06x5               g053(.a(new_n143), .b(new_n138), .c(new_n144), .d(new_n137), .out0(new_n149));
  nand23aa1n04x5               g054(.a(new_n149), .b(new_n128), .c(new_n130), .o1(new_n150));
  tech160nm_fioaoi03aa1n02p5x5 g055(.a(\a[12] ), .b(\b[11] ), .c(new_n142), .o1(new_n151));
  nanp02aa1n03x5               g056(.a(new_n133), .b(new_n131), .o1(new_n152));
  aoib12aa1n06x5               g057(.a(new_n151), .b(new_n149), .c(new_n152), .out0(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n150), .c(new_n147), .d(new_n148), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  xorc02aa1n12x5               g060(.a(\a[14] ), .b(\b[13] ), .out0(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  nor002aa1d24x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  tech160nm_fixorc02aa1n04x5   g063(.a(\a[13] ), .b(\b[12] ), .out0(new_n159));
  aoi112aa1n02x5               g064(.a(new_n158), .b(new_n156), .c(new_n154), .d(new_n159), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n158), .b(new_n154), .c(new_n159), .o1(new_n161));
  oab012aa1n02x4               g066(.a(new_n160), .b(new_n161), .c(new_n157), .out0(\s[14] ));
  and002aa1n02x5               g067(.a(new_n156), .b(new_n159), .o(new_n163));
  inv000aa1d42x5               g068(.a(\a[14] ), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\b[13] ), .o1(new_n165));
  oaoi03aa1n12x5               g070(.a(new_n164), .b(new_n165), .c(new_n158), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  nor002aa1n03x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand42aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n167), .c(new_n154), .d(new_n163), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(new_n171), .b(new_n167), .c(new_n154), .d(new_n163), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(\s[15] ));
  nor002aa1n03x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanp02aa1n09x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  aoib12aa1n02x5               g082(.a(new_n168), .b(new_n176), .c(new_n175), .out0(new_n178));
  oai012aa1n02x5               g083(.a(new_n172), .b(\b[14] ), .c(\a[15] ), .o1(new_n179));
  aoi022aa1n02x5               g084(.a(new_n179), .b(new_n177), .c(new_n172), .d(new_n178), .o1(\s[16] ));
  nano23aa1n03x7               g085(.a(new_n168), .b(new_n175), .c(new_n176), .d(new_n169), .out0(new_n181));
  nand23aa1n06x5               g086(.a(new_n181), .b(new_n159), .c(new_n156), .o1(new_n182));
  nor042aa1n04x5               g087(.a(new_n182), .b(new_n150), .o1(new_n183));
  aoai13aa1n12x5               g088(.a(new_n183), .b(new_n127), .c(new_n111), .d(new_n123), .o1(new_n184));
  nona23aa1n02x5               g089(.a(new_n137), .b(new_n144), .c(new_n143), .d(new_n138), .out0(new_n185));
  oabi12aa1n02x5               g090(.a(new_n151), .b(new_n185), .c(new_n152), .out0(new_n186));
  nona23aa1n03x5               g091(.a(new_n176), .b(new_n169), .c(new_n168), .d(new_n175), .out0(new_n187));
  aoi012aa1n02x5               g092(.a(new_n175), .b(new_n168), .c(new_n176), .o1(new_n188));
  oaih12aa1n02x5               g093(.a(new_n188), .b(new_n187), .c(new_n166), .o1(new_n189));
  aoib12aa1n09x5               g094(.a(new_n189), .b(new_n186), .c(new_n182), .out0(new_n190));
  xorc02aa1n12x5               g095(.a(\a[17] ), .b(\b[16] ), .out0(new_n191));
  xnbna2aa1n03x5               g096(.a(new_n191), .b(new_n184), .c(new_n190), .out0(\s[17] ));
  nand02aa1d08x5               g097(.a(new_n184), .b(new_n190), .o1(new_n193));
  nor002aa1n06x5               g098(.a(\b[16] ), .b(\a[17] ), .o1(new_n194));
  nor002aa1n06x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nand22aa1n06x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  norb02aa1n06x4               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  aoi112aa1n02x5               g102(.a(new_n194), .b(new_n197), .c(new_n193), .d(new_n191), .o1(new_n198));
  aoai13aa1n02x5               g103(.a(new_n197), .b(new_n194), .c(new_n193), .d(new_n191), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(\s[18] ));
  oao003aa1n02x5               g105(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n201));
  nano23aa1n02x4               g106(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n202));
  aobi12aa1n02x5               g107(.a(new_n110), .b(new_n202), .c(new_n201), .out0(new_n203));
  nanb02aa1n02x5               g108(.a(new_n116), .b(new_n124), .out0(new_n204));
  tech160nm_fioai012aa1n03p5x5 g109(.a(new_n148), .b(new_n203), .c(new_n204), .o1(new_n205));
  oabi12aa1n02x5               g110(.a(new_n189), .b(new_n153), .c(new_n182), .out0(new_n206));
  and002aa1n02x5               g111(.a(new_n191), .b(new_n197), .o(new_n207));
  aoai13aa1n02x5               g112(.a(new_n207), .b(new_n206), .c(new_n205), .d(new_n183), .o1(new_n208));
  aoi012aa1d24x5               g113(.a(new_n195), .b(new_n194), .c(new_n196), .o1(new_n209));
  nor002aa1d32x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nand02aa1d08x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nanb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  xnbna2aa1n03x5               g118(.a(new_n213), .b(new_n208), .c(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n06x5               g120(.a(new_n209), .o1(new_n216));
  aoai13aa1n06x5               g121(.a(new_n213), .b(new_n216), .c(new_n193), .d(new_n207), .o1(new_n217));
  nor002aa1d32x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand42aa1d28x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  aoib12aa1n02x5               g125(.a(new_n210), .b(new_n219), .c(new_n218), .out0(new_n221));
  oai012aa1n02x5               g126(.a(new_n217), .b(\b[18] ), .c(\a[19] ), .o1(new_n222));
  aoi022aa1n02x5               g127(.a(new_n222), .b(new_n220), .c(new_n217), .d(new_n221), .o1(\s[20] ));
  nano23aa1d15x5               g128(.a(new_n210), .b(new_n218), .c(new_n219), .d(new_n211), .out0(new_n224));
  nand23aa1d12x5               g129(.a(new_n224), .b(new_n191), .c(new_n197), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n206), .c(new_n205), .d(new_n183), .o1(new_n227));
  nona23aa1d18x5               g132(.a(new_n219), .b(new_n211), .c(new_n210), .d(new_n218), .out0(new_n228));
  oai012aa1n18x5               g133(.a(new_n219), .b(new_n218), .c(new_n210), .o1(new_n229));
  oai012aa1d24x5               g134(.a(new_n229), .b(new_n228), .c(new_n209), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[20] ), .b(\a[21] ), .out0(new_n232));
  xobna2aa1n03x5               g137(.a(new_n232), .b(new_n227), .c(new_n231), .out0(\s[21] ));
  tech160nm_fiao0012aa1n02p5x5 g138(.a(new_n232), .b(new_n227), .c(new_n231), .o(new_n234));
  xorc02aa1n12x5               g139(.a(\a[22] ), .b(\b[21] ), .out0(new_n235));
  nor042aa1n09x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  norp02aa1n02x5               g141(.a(new_n235), .b(new_n236), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n236), .o1(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n232), .c(new_n227), .d(new_n231), .o1(new_n239));
  aoi022aa1n02x5               g144(.a(new_n234), .b(new_n237), .c(new_n239), .d(new_n235), .o1(\s[22] ));
  nanb02aa1n06x5               g145(.a(new_n232), .b(new_n235), .out0(new_n241));
  nona22aa1n03x5               g146(.a(new_n193), .b(new_n225), .c(new_n241), .out0(new_n242));
  nanp02aa1n06x5               g147(.a(new_n224), .b(new_n216), .o1(new_n243));
  oaoi03aa1n12x5               g148(.a(\a[22] ), .b(\b[21] ), .c(new_n238), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  aoai13aa1n12x5               g150(.a(new_n245), .b(new_n241), .c(new_n243), .d(new_n229), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  xnrc02aa1n12x5               g152(.a(\b[22] ), .b(\a[23] ), .out0(new_n248));
  xobna2aa1n03x5               g153(.a(new_n248), .b(new_n242), .c(new_n247), .out0(\s[23] ));
  ao0012aa1n03x7               g154(.a(new_n248), .b(new_n242), .c(new_n247), .o(new_n250));
  xorc02aa1n02x5               g155(.a(\a[24] ), .b(\b[23] ), .out0(new_n251));
  nor042aa1n04x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  norp02aa1n02x5               g157(.a(new_n251), .b(new_n252), .o1(new_n253));
  inv000aa1n03x5               g158(.a(new_n252), .o1(new_n254));
  aoai13aa1n02x7               g159(.a(new_n254), .b(new_n248), .c(new_n242), .d(new_n247), .o1(new_n255));
  aoi022aa1n03x5               g160(.a(new_n250), .b(new_n253), .c(new_n255), .d(new_n251), .o1(\s[24] ));
  norb02aa1n06x4               g161(.a(new_n251), .b(new_n248), .out0(new_n257));
  norb03aa1n03x5               g162(.a(new_n257), .b(new_n225), .c(new_n241), .out0(new_n258));
  norb02aa1n02x5               g163(.a(new_n235), .b(new_n232), .out0(new_n259));
  aoai13aa1n03x5               g164(.a(new_n257), .b(new_n244), .c(new_n230), .d(new_n259), .o1(new_n260));
  tech160nm_fioaoi03aa1n02p5x5 g165(.a(\a[24] ), .b(\b[23] ), .c(new_n254), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(new_n260), .b(new_n262), .o1(new_n263));
  xorc02aa1n02x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n263), .c(new_n193), .d(new_n258), .o1(new_n265));
  aoi112aa1n02x5               g170(.a(new_n264), .b(new_n263), .c(new_n193), .d(new_n258), .o1(new_n266));
  norb02aa1n02x5               g171(.a(new_n265), .b(new_n266), .out0(\s[25] ));
  xorc02aa1n02x5               g172(.a(\a[26] ), .b(\b[25] ), .out0(new_n268));
  norp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  norp02aa1n02x5               g174(.a(new_n268), .b(new_n269), .o1(new_n270));
  inv000aa1d42x5               g175(.a(\a[25] ), .o1(new_n271));
  oaib12aa1n06x5               g176(.a(new_n265), .b(\b[24] ), .c(new_n271), .out0(new_n272));
  aoi022aa1n03x5               g177(.a(new_n272), .b(new_n268), .c(new_n265), .d(new_n270), .o1(\s[26] ));
  inv040aa1d32x5               g178(.a(\a[26] ), .o1(new_n274));
  xroi22aa1d06x4               g179(.a(new_n271), .b(\b[24] ), .c(new_n274), .d(\b[25] ), .out0(new_n275));
  nano32aa1n03x7               g180(.a(new_n225), .b(new_n275), .c(new_n259), .d(new_n257), .out0(new_n276));
  aobi12aa1n12x5               g181(.a(new_n276), .b(new_n184), .c(new_n190), .out0(new_n277));
  inv000aa1d42x5               g182(.a(new_n275), .o1(new_n278));
  oai022aa1n02x5               g183(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n279));
  oaib12aa1n02x5               g184(.a(new_n279), .b(new_n274), .c(\b[25] ), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n278), .c(new_n260), .d(new_n262), .o1(new_n281));
  xorc02aa1n02x5               g186(.a(\a[27] ), .b(\b[26] ), .out0(new_n282));
  oai012aa1n06x5               g187(.a(new_n282), .b(new_n281), .c(new_n277), .o1(new_n283));
  aoi112aa1n02x5               g188(.a(new_n281), .b(new_n282), .c(new_n193), .d(new_n276), .o1(new_n284));
  norb02aa1n03x4               g189(.a(new_n283), .b(new_n284), .out0(\s[27] ));
  xorc02aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .out0(new_n286));
  norp02aa1n02x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n286), .b(new_n287), .o1(new_n288));
  inv000aa1d42x5               g193(.a(\a[27] ), .o1(new_n289));
  oaib12aa1n06x5               g194(.a(new_n283), .b(\b[26] ), .c(new_n289), .out0(new_n290));
  aoi022aa1n02x7               g195(.a(new_n290), .b(new_n286), .c(new_n283), .d(new_n288), .o1(\s[28] ));
  inv000aa1d42x5               g196(.a(\a[28] ), .o1(new_n292));
  xroi22aa1d04x5               g197(.a(new_n289), .b(\b[26] ), .c(new_n292), .d(\b[27] ), .out0(new_n293));
  oaih12aa1n02x5               g198(.a(new_n293), .b(new_n281), .c(new_n277), .o1(new_n294));
  inv000aa1d42x5               g199(.a(\b[27] ), .o1(new_n295));
  oaoi03aa1n02x5               g200(.a(new_n292), .b(new_n295), .c(new_n287), .o1(new_n296));
  nanp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .out0(new_n298));
  norb02aa1n02x5               g203(.a(new_n296), .b(new_n298), .out0(new_n299));
  aoi022aa1n02x7               g204(.a(new_n297), .b(new_n298), .c(new_n294), .d(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g206(.a(new_n282), .b(new_n298), .c(new_n286), .o(new_n302));
  tech160nm_fioai012aa1n04x5   g207(.a(new_n302), .b(new_n281), .c(new_n277), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n304));
  nanp02aa1n03x5               g209(.a(new_n303), .b(new_n304), .o1(new_n305));
  xorc02aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .out0(new_n306));
  aoi012aa1n02x5               g211(.a(new_n296), .b(\a[29] ), .c(\b[28] ), .o1(new_n307));
  oabi12aa1n02x5               g212(.a(new_n306), .b(\a[29] ), .c(\b[28] ), .out0(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n307), .o1(new_n309));
  aoi022aa1n03x5               g214(.a(new_n305), .b(new_n306), .c(new_n303), .d(new_n309), .o1(\s[30] ));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  and003aa1n02x5               g216(.a(new_n293), .b(new_n306), .c(new_n298), .o(new_n312));
  oaih12aa1n02x5               g217(.a(new_n312), .b(new_n281), .c(new_n277), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n314));
  tech160nm_fiaoi012aa1n02p5x5 g219(.a(new_n311), .b(new_n313), .c(new_n314), .o1(new_n315));
  aoai13aa1n02x5               g220(.a(new_n276), .b(new_n206), .c(new_n205), .d(new_n183), .o1(new_n316));
  aoai13aa1n03x5               g221(.a(new_n275), .b(new_n261), .c(new_n246), .d(new_n257), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n312), .o1(new_n318));
  aoi013aa1n03x5               g223(.a(new_n318), .b(new_n316), .c(new_n317), .d(new_n280), .o1(new_n319));
  nano22aa1n03x5               g224(.a(new_n319), .b(new_n311), .c(new_n314), .out0(new_n320));
  norp02aa1n03x5               g225(.a(new_n315), .b(new_n320), .o1(\s[31] ));
  xorb03aa1n02x5               g226(.a(new_n102), .b(\b[2] ), .c(new_n108), .out0(\s[3] ));
  nanb03aa1n02x5               g227(.a(new_n105), .b(new_n201), .c(new_n106), .out0(new_n323));
  aboi22aa1n03x5               g228(.a(new_n103), .b(new_n104), .c(new_n108), .d(new_n109), .out0(new_n324));
  aboi22aa1n03x5               g229(.a(new_n103), .b(new_n111), .c(new_n323), .d(new_n324), .out0(\s[4] ));
  xorb03aa1n02x5               g230(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g231(.a(\a[5] ), .b(\b[4] ), .c(new_n203), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoi012aa1n02x5               g233(.a(new_n113), .b(new_n327), .c(new_n112), .o1(new_n329));
  xnrb03aa1n02x5               g234(.a(new_n329), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g235(.a(\a[7] ), .b(\b[6] ), .c(new_n329), .o1(new_n331));
  obai22aa1n02x7               g236(.a(new_n118), .b(new_n117), .c(\a[7] ), .d(\b[6] ), .out0(new_n332));
  oab012aa1n02x4               g237(.a(new_n332), .b(new_n329), .c(new_n122), .out0(new_n333));
  aoib12aa1n02x5               g238(.a(new_n333), .b(new_n331), .c(new_n119), .out0(\s[8] ));
  xnbna2aa1n03x5               g239(.a(new_n128), .b(new_n147), .c(new_n148), .out0(\s[9] ));
endmodule


