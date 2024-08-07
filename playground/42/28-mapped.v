// Benchmark "adder" written by ABC on Thu Jul 18 09:43:38 2024

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
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n311, new_n313, new_n316, new_n318, new_n320;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[10] ), .o1(new_n97));
  tech160nm_finand02aa1n03p5x5 g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nor042aa1n06x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  and002aa1n06x5               g004(.a(\b[3] ), .b(\a[4] ), .o(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  nand42aa1n06x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nor042aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nona22aa1n02x4               g009(.a(new_n103), .b(new_n104), .c(new_n102), .out0(new_n105));
  nor022aa1n16x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand42aa1n03x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nano22aa1n02x4               g012(.a(new_n106), .b(new_n103), .c(new_n107), .out0(new_n108));
  oab012aa1n06x5               g013(.a(new_n106), .b(\a[4] ), .c(\b[3] ), .out0(new_n109));
  inv000aa1d42x5               g014(.a(new_n109), .o1(new_n110));
  aoai13aa1n06x5               g015(.a(new_n101), .b(new_n110), .c(new_n108), .d(new_n105), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\a[7] ), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\b[6] ), .o1(new_n113));
  nor022aa1n06x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  aoi012aa1n09x5               g019(.a(new_n114), .b(new_n112), .c(new_n113), .o1(new_n115));
  aoi022aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  nor042aa1n04x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  nanb02aa1n02x5               g024(.a(new_n118), .b(new_n119), .out0(new_n120));
  nano23aa1n03x7               g025(.a(new_n120), .b(new_n117), .c(new_n115), .d(new_n116), .out0(new_n121));
  aoi112aa1n09x5               g026(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n122));
  oaih22aa1n04x5               g027(.a(new_n122), .b(new_n118), .c(new_n113), .d(new_n112), .o1(new_n123));
  aoi022aa1n12x5               g028(.a(new_n123), .b(new_n115), .c(\a[8] ), .d(\b[7] ), .o1(new_n124));
  inv000aa1d42x5               g029(.a(new_n124), .o1(new_n125));
  oaib12aa1n09x5               g030(.a(new_n125), .b(new_n111), .c(new_n121), .out0(new_n126));
  tech160nm_fioai012aa1n05x5   g031(.a(new_n98), .b(new_n126), .c(new_n99), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  and002aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o(new_n129));
  xorc02aa1n12x5               g034(.a(\a[10] ), .b(\b[9] ), .out0(new_n130));
  nor002aa1d32x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand02aa1n06x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n129), .c(new_n127), .d(new_n130), .o1(new_n134));
  aoi112aa1n06x5               g039(.a(new_n133), .b(new_n129), .c(new_n127), .d(new_n130), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(\s[11] ));
  inv000aa1d42x5               g041(.a(new_n131), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(new_n127), .b(new_n130), .o1(new_n138));
  nona22aa1n02x4               g043(.a(new_n138), .b(new_n133), .c(new_n129), .out0(new_n139));
  nor002aa1n06x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand02aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(new_n142));
  aoi012aa1n02x5               g047(.a(new_n142), .b(new_n139), .c(new_n137), .o1(new_n143));
  nano22aa1n02x4               g048(.a(new_n135), .b(new_n137), .c(new_n142), .out0(new_n144));
  norp02aa1n02x5               g049(.a(new_n143), .b(new_n144), .o1(\s[12] ));
  norb03aa1n03x5               g050(.a(new_n103), .b(new_n102), .c(new_n104), .out0(new_n146));
  nanb03aa1n03x5               g051(.a(new_n106), .b(new_n107), .c(new_n103), .out0(new_n147));
  oaoi13aa1n04x5               g052(.a(new_n100), .b(new_n109), .c(new_n146), .d(new_n147), .o1(new_n148));
  nano23aa1n06x5               g053(.a(new_n131), .b(new_n140), .c(new_n141), .d(new_n132), .out0(new_n149));
  norb02aa1n06x4               g054(.a(new_n98), .b(new_n99), .out0(new_n150));
  nand23aa1d12x5               g055(.a(new_n149), .b(new_n130), .c(new_n150), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n124), .c(new_n148), .d(new_n121), .o1(new_n153));
  nona23aa1n09x5               g058(.a(new_n141), .b(new_n132), .c(new_n131), .d(new_n140), .out0(new_n154));
  inv000aa1d42x5               g059(.a(\b[9] ), .o1(new_n155));
  oaoi03aa1n02x5               g060(.a(new_n97), .b(new_n155), .c(new_n99), .o1(new_n156));
  aoi012aa1n02x7               g061(.a(new_n140), .b(new_n131), .c(new_n141), .o1(new_n157));
  oai012aa1n12x5               g062(.a(new_n157), .b(new_n154), .c(new_n156), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  xnrc02aa1n12x5               g064(.a(\b[12] ), .b(\a[13] ), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n153), .c(new_n159), .out0(\s[13] ));
  orn002aa1n03x5               g067(.a(\a[13] ), .b(\b[12] ), .o(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n160), .c(new_n153), .d(new_n159), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xorc02aa1n02x5               g070(.a(\a[14] ), .b(\b[13] ), .out0(new_n166));
  norb02aa1n06x5               g071(.a(new_n166), .b(new_n160), .out0(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  oaoi03aa1n12x5               g073(.a(\a[14] ), .b(\b[13] ), .c(new_n163), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n168), .c(new_n153), .d(new_n159), .o1(new_n171));
  xorb03aa1n02x5               g076(.a(new_n171), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nand42aa1n03x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nor042aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nand42aa1n03x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  aoi112aa1n02x5               g082(.a(new_n173), .b(new_n177), .c(new_n171), .d(new_n174), .o1(new_n178));
  aoai13aa1n02x5               g083(.a(new_n177), .b(new_n173), .c(new_n171), .d(new_n174), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(\s[16] ));
  nano23aa1n06x5               g085(.a(new_n173), .b(new_n175), .c(new_n176), .d(new_n174), .out0(new_n181));
  nano32aa1n03x7               g086(.a(new_n151), .b(new_n181), .c(new_n161), .d(new_n166), .out0(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n124), .c(new_n148), .d(new_n121), .o1(new_n183));
  aoi012aa1n02x5               g088(.a(new_n175), .b(new_n173), .c(new_n176), .o1(new_n184));
  aob012aa1n03x5               g089(.a(new_n184), .b(new_n181), .c(new_n169), .out0(new_n185));
  aoi013aa1n06x4               g090(.a(new_n185), .b(new_n158), .c(new_n167), .d(new_n181), .o1(new_n186));
  nanp02aa1n09x5               g091(.a(new_n183), .b(new_n186), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g093(.a(\a[18] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d04x5               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  nanp02aa1n02x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nona22aa1n02x4               g100(.a(new_n195), .b(\b[16] ), .c(\a[17] ), .out0(new_n196));
  oaib12aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n189), .out0(new_n197));
  nor042aa1n03x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  aoai13aa1n06x5               g105(.a(new_n200), .b(new_n197), .c(new_n187), .d(new_n194), .o1(new_n201));
  aoi112aa1n02x5               g106(.a(new_n200), .b(new_n197), .c(new_n187), .d(new_n194), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n03x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nand02aa1n03x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  nona22aa1n03x5               g112(.a(new_n201), .b(new_n207), .c(new_n198), .out0(new_n208));
  orn002aa1n02x5               g113(.a(\a[19] ), .b(\b[18] ), .o(new_n209));
  aobi12aa1n03x5               g114(.a(new_n207), .b(new_n201), .c(new_n209), .out0(new_n210));
  norb02aa1n03x4               g115(.a(new_n208), .b(new_n210), .out0(\s[20] ));
  nano23aa1n06x5               g116(.a(new_n198), .b(new_n205), .c(new_n206), .d(new_n199), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n194), .b(new_n212), .o1(new_n213));
  norp02aa1n02x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  aoi013aa1n06x4               g119(.a(new_n214), .b(new_n195), .c(new_n190), .d(new_n191), .o1(new_n215));
  nona23aa1n09x5               g120(.a(new_n206), .b(new_n199), .c(new_n198), .d(new_n205), .out0(new_n216));
  aoi012aa1n12x5               g121(.a(new_n205), .b(new_n198), .c(new_n206), .o1(new_n217));
  oai012aa1d24x5               g122(.a(new_n217), .b(new_n216), .c(new_n215), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n213), .c(new_n183), .d(new_n186), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  tech160nm_finor002aa1n05x5   g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  nor042aa1n02x5               g129(.a(\b[21] ), .b(\a[22] ), .o1(new_n225));
  nand22aa1n02x5               g130(.a(\b[21] ), .b(\a[22] ), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(new_n227));
  aoi112aa1n02x5               g132(.a(new_n222), .b(new_n227), .c(new_n220), .d(new_n224), .o1(new_n228));
  aoai13aa1n03x5               g133(.a(new_n227), .b(new_n222), .c(new_n220), .d(new_n224), .o1(new_n229));
  norb02aa1n02x5               g134(.a(new_n229), .b(new_n228), .out0(\s[22] ));
  nano23aa1n06x5               g135(.a(new_n222), .b(new_n225), .c(new_n226), .d(new_n223), .out0(new_n231));
  nanp03aa1n02x5               g136(.a(new_n194), .b(new_n212), .c(new_n231), .o1(new_n232));
  aoi012aa1d18x5               g137(.a(new_n225), .b(new_n222), .c(new_n226), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aoi012aa1n02x5               g139(.a(new_n234), .b(new_n218), .c(new_n231), .o1(new_n235));
  aoai13aa1n06x5               g140(.a(new_n235), .b(new_n232), .c(new_n183), .d(new_n186), .o1(new_n236));
  xorb03aa1n02x5               g141(.a(new_n236), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  tech160nm_fixorc02aa1n02p5x5 g143(.a(\a[23] ), .b(\b[22] ), .out0(new_n239));
  xorc02aa1n02x5               g144(.a(\a[24] ), .b(\b[23] ), .out0(new_n240));
  aoi112aa1n02x5               g145(.a(new_n238), .b(new_n240), .c(new_n236), .d(new_n239), .o1(new_n241));
  aoai13aa1n03x5               g146(.a(new_n240), .b(new_n238), .c(new_n236), .d(new_n239), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(\s[24] ));
  inv040aa1n02x5               g148(.a(new_n217), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n231), .b(new_n244), .c(new_n212), .d(new_n197), .o1(new_n245));
  and002aa1n06x5               g150(.a(new_n240), .b(new_n239), .o(new_n246));
  inv000aa1n02x5               g151(.a(new_n246), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n248));
  oab012aa1n02x4               g153(.a(new_n248), .b(\a[24] ), .c(\b[23] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n247), .c(new_n245), .d(new_n233), .o1(new_n250));
  nano32aa1n02x4               g155(.a(new_n213), .b(new_n240), .c(new_n231), .d(new_n239), .out0(new_n251));
  xorc02aa1n02x5               g156(.a(\a[25] ), .b(\b[24] ), .out0(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n250), .c(new_n187), .d(new_n251), .o1(new_n253));
  aoi112aa1n02x5               g158(.a(new_n250), .b(new_n252), .c(new_n187), .d(new_n251), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n253), .b(new_n254), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[26] ), .b(\b[25] ), .out0(new_n257));
  nona22aa1n02x4               g162(.a(new_n253), .b(new_n257), .c(new_n256), .out0(new_n258));
  inv000aa1n02x5               g163(.a(new_n256), .o1(new_n259));
  aobi12aa1n02x7               g164(.a(new_n257), .b(new_n253), .c(new_n259), .out0(new_n260));
  norb02aa1n03x4               g165(.a(new_n258), .b(new_n260), .out0(\s[26] ));
  nanp03aa1n02x5               g166(.a(new_n161), .b(new_n181), .c(new_n166), .o1(new_n262));
  oabi12aa1n02x5               g167(.a(new_n185), .b(new_n159), .c(new_n262), .out0(new_n263));
  inv000aa1d42x5               g168(.a(\a[25] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(\a[26] ), .o1(new_n265));
  xroi22aa1d06x4               g170(.a(new_n264), .b(\b[24] ), .c(new_n265), .d(\b[25] ), .out0(new_n266));
  nano22aa1n03x7               g171(.a(new_n232), .b(new_n246), .c(new_n266), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n263), .c(new_n126), .d(new_n182), .o1(new_n268));
  oao003aa1n02x5               g173(.a(\a[26] ), .b(\b[25] ), .c(new_n259), .carry(new_n269));
  aobi12aa1n06x5               g174(.a(new_n269), .b(new_n250), .c(new_n266), .out0(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  xnbna2aa1n03x5               g176(.a(new_n271), .b(new_n270), .c(new_n268), .out0(\s[27] ));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  inv040aa1n03x5               g178(.a(new_n273), .o1(new_n274));
  aobi12aa1n02x5               g179(.a(new_n271), .b(new_n270), .c(new_n268), .out0(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  nano22aa1n03x5               g181(.a(new_n275), .b(new_n274), .c(new_n276), .out0(new_n277));
  inv020aa1n03x5               g182(.a(new_n267), .o1(new_n278));
  tech160nm_fiaoi012aa1n05x5   g183(.a(new_n278), .b(new_n183), .c(new_n186), .o1(new_n279));
  aoai13aa1n06x5               g184(.a(new_n246), .b(new_n234), .c(new_n218), .d(new_n231), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n266), .o1(new_n281));
  aoai13aa1n12x5               g186(.a(new_n269), .b(new_n281), .c(new_n280), .d(new_n249), .o1(new_n282));
  oaih12aa1n02x5               g187(.a(new_n271), .b(new_n282), .c(new_n279), .o1(new_n283));
  tech160nm_fiaoi012aa1n02p5x5 g188(.a(new_n276), .b(new_n283), .c(new_n274), .o1(new_n284));
  nor002aa1n02x5               g189(.a(new_n284), .b(new_n277), .o1(\s[28] ));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  norb02aa1n02x5               g191(.a(new_n271), .b(new_n276), .out0(new_n287));
  aobi12aa1n06x5               g192(.a(new_n287), .b(new_n270), .c(new_n268), .out0(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n274), .carry(new_n289));
  nano22aa1n02x4               g194(.a(new_n288), .b(new_n286), .c(new_n289), .out0(new_n290));
  tech160nm_fioai012aa1n03p5x5 g195(.a(new_n287), .b(new_n282), .c(new_n279), .o1(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n286), .b(new_n291), .c(new_n289), .o1(new_n292));
  norp02aa1n02x5               g197(.a(new_n292), .b(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g199(.a(new_n271), .b(new_n286), .c(new_n276), .out0(new_n295));
  aobi12aa1n02x5               g200(.a(new_n295), .b(new_n270), .c(new_n268), .out0(new_n296));
  oao003aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[29] ), .b(\a[30] ), .out0(new_n298));
  nano22aa1n03x5               g203(.a(new_n296), .b(new_n297), .c(new_n298), .out0(new_n299));
  oaih12aa1n02x5               g204(.a(new_n295), .b(new_n282), .c(new_n279), .o1(new_n300));
  tech160nm_fiaoi012aa1n02p5x5 g205(.a(new_n298), .b(new_n300), .c(new_n297), .o1(new_n301));
  nor002aa1n02x5               g206(.a(new_n301), .b(new_n299), .o1(\s[30] ));
  norb02aa1n02x5               g207(.a(new_n295), .b(new_n298), .out0(new_n303));
  aobi12aa1n06x5               g208(.a(new_n303), .b(new_n270), .c(new_n268), .out0(new_n304));
  oao003aa1n02x5               g209(.a(\a[30] ), .b(\b[29] ), .c(new_n297), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  nano22aa1n03x5               g211(.a(new_n304), .b(new_n305), .c(new_n306), .out0(new_n307));
  oaih12aa1n02x5               g212(.a(new_n303), .b(new_n282), .c(new_n279), .o1(new_n308));
  aoi012aa1n03x5               g213(.a(new_n306), .b(new_n308), .c(new_n305), .o1(new_n309));
  norp02aa1n02x5               g214(.a(new_n309), .b(new_n307), .o1(\s[31] ));
  norb02aa1n02x5               g215(.a(new_n107), .b(new_n106), .out0(new_n311));
  xobna2aa1n03x5               g216(.a(new_n311), .b(new_n105), .c(new_n103), .out0(\s[3] ));
  oabi12aa1n02x5               g217(.a(new_n106), .b(new_n146), .c(new_n147), .out0(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnrb03aa1n02x5               g219(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g220(.a(\a[5] ), .b(\b[4] ), .c(new_n111), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oai012aa1n02x5               g222(.a(new_n119), .b(new_n316), .c(new_n118), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[6] ), .c(new_n112), .out0(\s[7] ));
  oaoi03aa1n02x5               g224(.a(\a[7] ), .b(\b[6] ), .c(new_n318), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g226(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


