// Benchmark "adder" written by ABC on Thu Jul 18 06:20:40 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n319, new_n322, new_n324, new_n326;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nand42aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand22aa1n04x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nor002aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  oaih12aa1n06x5               g005(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n101));
  norp02aa1n12x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand42aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nor022aa1n16x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand02aa1d06x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nona23aa1n09x5               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  aoi012aa1n06x5               g011(.a(new_n104), .b(new_n102), .c(new_n105), .o1(new_n107));
  oai012aa1n12x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor002aa1n06x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nanp02aa1n04x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nor002aa1d32x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand02aa1d08x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  nand02aa1n12x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nor042aa1n04x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nanp02aa1n24x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nano22aa1n12x5               g022(.a(new_n116), .b(new_n115), .c(new_n117), .out0(new_n118));
  norb03aa1n06x5               g023(.a(new_n118), .b(new_n114), .c(new_n109), .out0(new_n119));
  nand02aa1d04x5               g024(.a(new_n119), .b(new_n108), .o1(new_n120));
  norb02aa1n06x5               g025(.a(new_n113), .b(new_n112), .out0(new_n121));
  aoi112aa1n02x7               g026(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n122));
  oai022aa1d18x5               g027(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n123));
  aoi113aa1n03x7               g028(.a(new_n122), .b(new_n112), .c(new_n118), .d(new_n121), .e(new_n123), .o1(new_n124));
  nand02aa1n06x5               g029(.a(new_n120), .b(new_n124), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  xnrc02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n97), .c(new_n125), .d(new_n126), .o1(new_n128));
  nanp03aa1n02x5               g033(.a(new_n118), .b(new_n121), .c(new_n123), .o1(new_n129));
  nona22aa1n03x5               g034(.a(new_n129), .b(new_n122), .c(new_n112), .out0(new_n130));
  norb02aa1n02x5               g035(.a(new_n126), .b(new_n97), .out0(new_n131));
  aoai13aa1n03x5               g036(.a(new_n131), .b(new_n130), .c(new_n119), .d(new_n108), .o1(new_n132));
  nona22aa1n03x5               g037(.a(new_n132), .b(new_n127), .c(new_n97), .out0(new_n133));
  nanp02aa1n02x5               g038(.a(new_n128), .b(new_n133), .o1(\s[10] ));
  and002aa1n12x5               g039(.a(\b[9] ), .b(\a[10] ), .o(new_n135));
  nand42aa1n20x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nor002aa1d32x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n06x4               g042(.a(new_n136), .b(new_n137), .out0(new_n138));
  aoib12aa1n02x5               g043(.a(new_n138), .b(new_n133), .c(new_n135), .out0(new_n139));
  nanb03aa1n03x5               g044(.a(new_n135), .b(new_n133), .c(new_n138), .out0(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(\s[11] ));
  inv000aa1d42x5               g046(.a(new_n137), .o1(new_n142));
  nor002aa1d32x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand42aa1d28x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n03x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n140), .c(new_n142), .out0(\s[12] ));
  nor042aa1n02x5               g051(.a(\b[9] ), .b(\a[10] ), .o1(new_n147));
  nor043aa1n02x5               g052(.a(new_n135), .b(new_n147), .c(new_n97), .o1(new_n148));
  nona23aa1n09x5               g053(.a(new_n136), .b(new_n144), .c(new_n143), .d(new_n137), .out0(new_n149));
  nanb03aa1n02x5               g054(.a(new_n149), .b(new_n126), .c(new_n148), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n143), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n137), .b(new_n144), .o1(new_n152));
  oabi12aa1n03x5               g057(.a(new_n135), .b(new_n97), .c(new_n147), .out0(new_n153));
  nor002aa1n02x5               g058(.a(new_n149), .b(new_n153), .o1(new_n154));
  nano22aa1n02x5               g059(.a(new_n154), .b(new_n151), .c(new_n152), .out0(new_n155));
  aoai13aa1n06x5               g060(.a(new_n155), .b(new_n150), .c(new_n120), .d(new_n124), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(\a[14] ), .o1(new_n158));
  nor002aa1d32x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nand42aa1d28x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  tech160nm_fiaoi012aa1n05x5   g065(.a(new_n159), .b(new_n156), .c(new_n160), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(new_n158), .out0(\s[14] ));
  norb02aa1n03x5               g067(.a(new_n160), .b(new_n159), .out0(new_n163));
  tech160nm_fixorc02aa1n05x5   g068(.a(\a[14] ), .b(\b[13] ), .out0(new_n164));
  and002aa1n02x5               g069(.a(new_n164), .b(new_n163), .o(new_n165));
  inv040aa1n08x5               g070(.a(new_n159), .o1(new_n166));
  oao003aa1n12x5               g071(.a(\a[14] ), .b(\b[13] ), .c(new_n166), .carry(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  nor002aa1d24x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nand02aa1n16x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  norb02aa1n15x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n168), .c(new_n156), .d(new_n165), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(new_n171), .b(new_n168), .c(new_n156), .d(new_n165), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(\s[15] ));
  xorc02aa1n12x5               g079(.a(\a[16] ), .b(\b[15] ), .out0(new_n175));
  nona22aa1n02x4               g080(.a(new_n172), .b(new_n175), .c(new_n169), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n169), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n175), .o1(new_n178));
  aoi012aa1n02x7               g083(.a(new_n178), .b(new_n172), .c(new_n177), .o1(new_n179));
  norb02aa1n02x7               g084(.a(new_n176), .b(new_n179), .out0(\s[16] ));
  nand03aa1n03x5               g085(.a(new_n138), .b(new_n145), .c(new_n126), .o1(new_n181));
  nand03aa1n03x5               g086(.a(new_n164), .b(new_n163), .c(new_n171), .o1(new_n182));
  nano23aa1n09x5               g087(.a(new_n182), .b(new_n181), .c(new_n148), .d(new_n175), .out0(new_n183));
  aoai13aa1n06x5               g088(.a(new_n183), .b(new_n130), .c(new_n108), .d(new_n119), .o1(new_n184));
  oai112aa1n03x5               g089(.a(new_n151), .b(new_n152), .c(new_n149), .d(new_n153), .o1(new_n185));
  nano32aa1n03x7               g090(.a(new_n178), .b(new_n164), .c(new_n171), .d(new_n163), .out0(new_n186));
  inv000aa1d42x5               g091(.a(\a[16] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\b[15] ), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(new_n188), .b(new_n187), .o1(new_n189));
  aoi022aa1n06x5               g094(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n190));
  inv000aa1n02x5               g095(.a(new_n190), .o1(new_n191));
  aoai13aa1n09x5               g096(.a(new_n189), .b(new_n191), .c(new_n167), .d(new_n177), .o1(new_n192));
  aoi012aa1n12x5               g097(.a(new_n192), .b(new_n185), .c(new_n186), .o1(new_n193));
  xorc02aa1n02x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  xnbna2aa1n03x5               g099(.a(new_n194), .b(new_n184), .c(new_n193), .out0(\s[17] ));
  inv000aa1d42x5               g100(.a(\a[17] ), .o1(new_n196));
  inv000aa1d42x5               g101(.a(\b[16] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(new_n197), .b(new_n196), .o1(new_n198));
  nanp02aa1n03x5               g103(.a(new_n167), .b(new_n177), .o1(new_n199));
  aoi022aa1n02x7               g104(.a(new_n199), .b(new_n190), .c(new_n188), .d(new_n187), .o1(new_n200));
  oaib12aa1n06x5               g105(.a(new_n200), .b(new_n155), .c(new_n186), .out0(new_n201));
  aoai13aa1n02x5               g106(.a(new_n194), .b(new_n201), .c(new_n125), .d(new_n183), .o1(new_n202));
  nor002aa1d32x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  nand42aa1n16x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nanb02aa1n12x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  xobna2aa1n03x5               g110(.a(new_n205), .b(new_n202), .c(new_n198), .out0(\s[18] ));
  nanp02aa1n02x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  nano22aa1d15x5               g112(.a(new_n205), .b(new_n198), .c(new_n207), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoai13aa1n12x5               g114(.a(new_n204), .b(new_n203), .c(new_n196), .d(new_n197), .o1(new_n210));
  aoai13aa1n06x5               g115(.a(new_n210), .b(new_n209), .c(new_n184), .d(new_n193), .o1(new_n211));
  xorb03aa1n02x5               g116(.a(new_n211), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nand02aa1n10x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  nor002aa1d32x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nand02aa1d08x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(new_n218));
  aoi112aa1n02x5               g123(.a(new_n214), .b(new_n218), .c(new_n211), .d(new_n215), .o1(new_n219));
  aoai13aa1n03x5               g124(.a(new_n218), .b(new_n214), .c(new_n211), .d(new_n215), .o1(new_n220));
  norb02aa1n02x7               g125(.a(new_n220), .b(new_n219), .out0(\s[20] ));
  nano23aa1n03x7               g126(.a(new_n214), .b(new_n216), .c(new_n217), .d(new_n215), .out0(new_n222));
  nanp02aa1n02x5               g127(.a(new_n208), .b(new_n222), .o1(new_n223));
  nona23aa1d18x5               g128(.a(new_n217), .b(new_n215), .c(new_n214), .d(new_n216), .out0(new_n224));
  aoi012aa1n06x5               g129(.a(new_n216), .b(new_n214), .c(new_n217), .o1(new_n225));
  oai012aa1n12x5               g130(.a(new_n225), .b(new_n224), .c(new_n210), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n223), .c(new_n184), .d(new_n193), .o1(new_n228));
  xorb03aa1n02x5               g133(.a(new_n228), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g134(.a(\b[20] ), .b(\a[21] ), .o1(new_n230));
  xorc02aa1n02x5               g135(.a(\a[21] ), .b(\b[20] ), .out0(new_n231));
  xorc02aa1n02x5               g136(.a(\a[22] ), .b(\b[21] ), .out0(new_n232));
  aoi112aa1n02x5               g137(.a(new_n230), .b(new_n232), .c(new_n228), .d(new_n231), .o1(new_n233));
  aoai13aa1n03x5               g138(.a(new_n232), .b(new_n230), .c(new_n228), .d(new_n231), .o1(new_n234));
  norb02aa1n02x7               g139(.a(new_n234), .b(new_n233), .out0(\s[22] ));
  inv000aa1d42x5               g140(.a(\a[21] ), .o1(new_n236));
  inv040aa1d32x5               g141(.a(\a[22] ), .o1(new_n237));
  xroi22aa1d04x5               g142(.a(new_n236), .b(\b[20] ), .c(new_n237), .d(\b[21] ), .out0(new_n238));
  nand23aa1n03x5               g143(.a(new_n238), .b(new_n208), .c(new_n222), .o1(new_n239));
  inv000aa1d42x5               g144(.a(\b[21] ), .o1(new_n240));
  oaoi03aa1n09x5               g145(.a(new_n237), .b(new_n240), .c(new_n230), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  aoi012aa1n02x5               g147(.a(new_n242), .b(new_n226), .c(new_n238), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n239), .c(new_n184), .d(new_n193), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  xorc02aa1n03x5               g151(.a(\a[23] ), .b(\b[22] ), .out0(new_n247));
  xorc02aa1n02x5               g152(.a(\a[24] ), .b(\b[23] ), .out0(new_n248));
  aoi112aa1n02x5               g153(.a(new_n246), .b(new_n248), .c(new_n244), .d(new_n247), .o1(new_n249));
  aoai13aa1n03x5               g154(.a(new_n248), .b(new_n246), .c(new_n244), .d(new_n247), .o1(new_n250));
  norb02aa1n02x7               g155(.a(new_n250), .b(new_n249), .out0(\s[24] ));
  nanp02aa1n06x5               g156(.a(new_n184), .b(new_n193), .o1(new_n252));
  and002aa1n02x5               g157(.a(new_n248), .b(new_n247), .o(new_n253));
  inv000aa1n02x5               g158(.a(new_n253), .o1(new_n254));
  nor002aa1n02x5               g159(.a(new_n239), .b(new_n254), .o1(new_n255));
  inv030aa1n02x5               g160(.a(new_n210), .o1(new_n256));
  inv000aa1n03x5               g161(.a(new_n225), .o1(new_n257));
  aoai13aa1n06x5               g162(.a(new_n238), .b(new_n257), .c(new_n222), .d(new_n256), .o1(new_n258));
  orn002aa1n02x5               g163(.a(\a[23] ), .b(\b[22] ), .o(new_n259));
  oao003aa1n02x5               g164(.a(\a[24] ), .b(\b[23] ), .c(new_n259), .carry(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n254), .c(new_n258), .d(new_n241), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n261), .c(new_n252), .d(new_n255), .o1(new_n263));
  aoi112aa1n02x5               g168(.a(new_n262), .b(new_n261), .c(new_n252), .d(new_n255), .o1(new_n264));
  norb02aa1n02x7               g169(.a(new_n263), .b(new_n264), .out0(\s[25] ));
  nor042aa1n03x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  tech160nm_fixorc02aa1n05x5   g171(.a(\a[26] ), .b(\b[25] ), .out0(new_n267));
  nona22aa1n02x5               g172(.a(new_n263), .b(new_n267), .c(new_n266), .out0(new_n268));
  inv000aa1d42x5               g173(.a(new_n266), .o1(new_n269));
  aobi12aa1n02x7               g174(.a(new_n267), .b(new_n263), .c(new_n269), .out0(new_n270));
  norb02aa1n03x4               g175(.a(new_n268), .b(new_n270), .out0(\s[26] ));
  and002aa1n09x5               g176(.a(new_n267), .b(new_n262), .o(new_n272));
  nano22aa1n03x7               g177(.a(new_n239), .b(new_n253), .c(new_n272), .out0(new_n273));
  aoai13aa1n04x5               g178(.a(new_n273), .b(new_n201), .c(new_n125), .d(new_n183), .o1(new_n274));
  oao003aa1n02x5               g179(.a(\a[26] ), .b(\b[25] ), .c(new_n269), .carry(new_n275));
  aobi12aa1n06x5               g180(.a(new_n275), .b(new_n261), .c(new_n272), .out0(new_n276));
  xorc02aa1n12x5               g181(.a(\a[27] ), .b(\b[26] ), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n277), .b(new_n276), .c(new_n274), .out0(\s[27] ));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  inv040aa1n03x5               g184(.a(new_n279), .o1(new_n280));
  aobi12aa1n02x7               g185(.a(new_n277), .b(new_n276), .c(new_n274), .out0(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[27] ), .b(\a[28] ), .out0(new_n282));
  nano22aa1n02x4               g187(.a(new_n281), .b(new_n280), .c(new_n282), .out0(new_n283));
  inv000aa1n02x5               g188(.a(new_n273), .o1(new_n284));
  aoi012aa1n06x5               g189(.a(new_n284), .b(new_n184), .c(new_n193), .o1(new_n285));
  aoai13aa1n02x7               g190(.a(new_n253), .b(new_n242), .c(new_n226), .d(new_n238), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n272), .o1(new_n287));
  aoai13aa1n06x5               g192(.a(new_n275), .b(new_n287), .c(new_n286), .d(new_n260), .o1(new_n288));
  oaih12aa1n02x5               g193(.a(new_n277), .b(new_n288), .c(new_n285), .o1(new_n289));
  aoi012aa1n03x5               g194(.a(new_n282), .b(new_n289), .c(new_n280), .o1(new_n290));
  nor002aa1n02x5               g195(.a(new_n290), .b(new_n283), .o1(\s[28] ));
  norb02aa1n02x5               g196(.a(new_n277), .b(new_n282), .out0(new_n292));
  oaih12aa1n02x5               g197(.a(new_n292), .b(new_n288), .c(new_n285), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .c(new_n280), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[28] ), .b(\a[29] ), .out0(new_n295));
  aoi012aa1n03x5               g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n02x7               g201(.a(new_n292), .b(new_n276), .c(new_n274), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  nor002aa1n02x5               g203(.a(new_n296), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g205(.a(new_n277), .b(new_n295), .c(new_n282), .out0(new_n301));
  oaih12aa1n02x5               g206(.a(new_n301), .b(new_n288), .c(new_n285), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[29] ), .b(\a[30] ), .out0(new_n304));
  aoi012aa1n03x5               g209(.a(new_n304), .b(new_n302), .c(new_n303), .o1(new_n305));
  aobi12aa1n02x7               g210(.a(new_n301), .b(new_n276), .c(new_n274), .out0(new_n306));
  nano22aa1n02x4               g211(.a(new_n306), .b(new_n303), .c(new_n304), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n305), .b(new_n307), .o1(\s[30] ));
  xnrc02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  norb02aa1n02x5               g214(.a(new_n301), .b(new_n304), .out0(new_n310));
  oaih12aa1n02x5               g215(.a(new_n310), .b(new_n288), .c(new_n285), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .carry(new_n312));
  aoi012aa1n03x5               g217(.a(new_n309), .b(new_n311), .c(new_n312), .o1(new_n313));
  aobi12aa1n02x7               g218(.a(new_n310), .b(new_n276), .c(new_n274), .out0(new_n314));
  nano22aa1n02x4               g219(.a(new_n314), .b(new_n309), .c(new_n312), .out0(new_n315));
  norp02aa1n03x5               g220(.a(new_n313), .b(new_n315), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nano22aa1n02x4               g222(.a(new_n102), .b(new_n101), .c(new_n103), .out0(new_n318));
  oaib12aa1n02x5               g223(.a(new_n103), .b(new_n104), .c(new_n105), .out0(new_n319));
  obai22aa1n02x7               g224(.a(new_n105), .b(new_n108), .c(new_n318), .d(new_n319), .out0(\s[4] ));
  xorb03aa1n02x5               g225(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g226(.a(new_n111), .b(new_n108), .c(new_n110), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n03x5               g228(.a(\a[6] ), .b(\b[5] ), .c(new_n322), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oai012aa1n03x5               g230(.a(new_n115), .b(new_n324), .c(new_n116), .o1(new_n326));
  xnrc02aa1n02x5               g231(.a(new_n326), .b(new_n121), .out0(\s[8] ));
  xnbna2aa1n03x5               g232(.a(new_n131), .b(new_n120), .c(new_n124), .out0(\s[9] ));
endmodule


