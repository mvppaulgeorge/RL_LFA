// Benchmark "adder" written by ABC on Thu Jul 18 09:16:38 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n278, new_n279, new_n280, new_n281,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n311, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n334, new_n336, new_n337, new_n339,
    new_n341, new_n342, new_n344, new_n345, new_n347, new_n349;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand42aa1n04x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nor022aa1n08x5               g002(.a(\b[2] ), .b(\a[3] ), .o1(new_n98));
  nand02aa1n06x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nano22aa1n03x7               g004(.a(new_n98), .b(new_n97), .c(new_n99), .out0(new_n100));
  oai112aa1n06x5               g005(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n101));
  nor042aa1n06x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n09x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n03x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nanp03aa1n03x5               g009(.a(new_n100), .b(new_n101), .c(new_n104), .o1(new_n105));
  tech160nm_fioai012aa1n05x5   g010(.a(new_n103), .b(new_n102), .c(new_n98), .o1(new_n106));
  nanp02aa1n04x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nor022aa1n16x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nor042aa1d18x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nand42aa1n04x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nano23aa1n03x7               g015(.a(new_n109), .b(new_n108), .c(new_n110), .d(new_n107), .out0(new_n111));
  nor022aa1n16x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand02aa1d24x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor042aa1n06x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand02aa1d12x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n09x5               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  nand02aa1n02x5               g021(.a(new_n116), .b(new_n111), .o1(new_n117));
  inv030aa1n06x5               g022(.a(new_n109), .o1(new_n118));
  oaoi03aa1n09x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  oa0012aa1n02x5               g024(.a(new_n113), .b(new_n114), .c(new_n112), .o(new_n120));
  aoi012aa1n09x5               g025(.a(new_n120), .b(new_n116), .c(new_n119), .o1(new_n121));
  aoai13aa1n06x5               g026(.a(new_n121), .b(new_n117), .c(new_n105), .d(new_n106), .o1(new_n122));
  xorc02aa1n12x5               g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  nor042aa1n04x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nor042aa1n04x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand02aa1d08x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb03aa1n02x5               g031(.a(new_n126), .b(new_n124), .c(new_n125), .out0(new_n127));
  aob012aa1n03x5               g032(.a(new_n127), .b(new_n122), .c(new_n123), .out0(new_n128));
  nanb02aa1n03x5               g033(.a(new_n125), .b(new_n126), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n124), .c(new_n122), .d(new_n123), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(new_n130), .b(new_n128), .o1(\s[10] ));
  nand42aa1n06x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nano22aa1n02x4               g038(.a(new_n133), .b(new_n126), .c(new_n132), .out0(new_n134));
  nanb02aa1n02x5               g039(.a(new_n133), .b(new_n132), .out0(new_n135));
  nanp02aa1n02x5               g040(.a(new_n128), .b(new_n126), .o1(new_n136));
  aoi022aa1n02x5               g041(.a(new_n136), .b(new_n135), .c(new_n128), .d(new_n134), .o1(\s[11] ));
  orn002aa1n02x5               g042(.a(\a[11] ), .b(\b[10] ), .o(new_n138));
  nanp02aa1n03x5               g043(.a(new_n128), .b(new_n134), .o1(new_n139));
  nor022aa1n16x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand22aa1n09x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n03x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n139), .c(new_n138), .out0(\s[12] ));
  nona23aa1d18x5               g048(.a(new_n132), .b(new_n141), .c(new_n140), .d(new_n133), .out0(new_n144));
  norb03aa1n03x5               g049(.a(new_n123), .b(new_n144), .c(new_n129), .out0(new_n145));
  nanp02aa1n02x5               g050(.a(new_n122), .b(new_n145), .o1(new_n146));
  nanb03aa1n12x5               g051(.a(new_n98), .b(new_n99), .c(new_n97), .out0(new_n147));
  inv000aa1n02x5               g052(.a(new_n102), .o1(new_n148));
  nand23aa1n06x5               g053(.a(new_n101), .b(new_n148), .c(new_n103), .o1(new_n149));
  oai012aa1n06x5               g054(.a(new_n106), .b(new_n149), .c(new_n147), .o1(new_n150));
  nona23aa1n09x5               g055(.a(new_n107), .b(new_n110), .c(new_n109), .d(new_n108), .out0(new_n151));
  norb02aa1n03x5               g056(.a(new_n113), .b(new_n112), .out0(new_n152));
  norb02aa1n03x5               g057(.a(new_n115), .b(new_n114), .out0(new_n153));
  nano22aa1n03x5               g058(.a(new_n151), .b(new_n152), .c(new_n153), .out0(new_n154));
  nand22aa1n06x5               g059(.a(new_n150), .b(new_n154), .o1(new_n155));
  nona23aa1n02x4               g060(.a(new_n123), .b(new_n142), .c(new_n135), .d(new_n129), .out0(new_n156));
  aoi012aa1n12x5               g061(.a(new_n125), .b(new_n124), .c(new_n126), .o1(new_n157));
  aoi012aa1n12x5               g062(.a(new_n140), .b(new_n133), .c(new_n141), .o1(new_n158));
  oai012aa1d24x5               g063(.a(new_n158), .b(new_n144), .c(new_n157), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n156), .c(new_n155), .d(new_n121), .o1(new_n161));
  nor002aa1d32x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nand42aa1n08x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nanb02aa1n18x5               g068(.a(new_n162), .b(new_n163), .out0(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  nona22aa1n03x5               g070(.a(new_n142), .b(new_n157), .c(new_n135), .out0(new_n166));
  and003aa1n02x5               g071(.a(new_n166), .b(new_n164), .c(new_n158), .o(new_n167));
  aoi022aa1n02x5               g072(.a(new_n161), .b(new_n165), .c(new_n146), .d(new_n167), .o1(\s[13] ));
  inv000aa1d42x5               g073(.a(new_n162), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n165), .b(new_n159), .c(new_n122), .d(new_n145), .o1(new_n170));
  nor042aa1n06x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nand22aa1n09x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nanb02aa1n03x5               g077(.a(new_n171), .b(new_n172), .out0(new_n173));
  xobna2aa1n03x5               g078(.a(new_n173), .b(new_n170), .c(new_n169), .out0(\s[14] ));
  nano23aa1n03x7               g079(.a(new_n162), .b(new_n171), .c(new_n172), .d(new_n163), .out0(new_n175));
  aoi012aa1n12x5               g080(.a(new_n171), .b(new_n162), .c(new_n172), .o1(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  nor022aa1n08x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nanp02aa1n04x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  aoai13aa1n03x5               g085(.a(new_n180), .b(new_n177), .c(new_n161), .d(new_n175), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(new_n180), .b(new_n177), .c(new_n161), .d(new_n175), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n181), .b(new_n182), .out0(\s[15] ));
  nor022aa1n08x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nanp02aa1n04x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  norb02aa1n02x5               g090(.a(new_n185), .b(new_n184), .out0(new_n186));
  aoib12aa1n02x5               g091(.a(new_n178), .b(new_n185), .c(new_n184), .out0(new_n187));
  oai012aa1n02x5               g092(.a(new_n181), .b(\b[14] ), .c(\a[15] ), .o1(new_n188));
  aoi022aa1n02x5               g093(.a(new_n188), .b(new_n186), .c(new_n181), .d(new_n187), .o1(\s[16] ));
  nano23aa1n03x7               g094(.a(new_n178), .b(new_n184), .c(new_n185), .d(new_n179), .out0(new_n190));
  nand42aa1n02x5               g095(.a(new_n190), .b(new_n175), .o1(new_n191));
  nor042aa1n02x5               g096(.a(new_n156), .b(new_n191), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(new_n122), .b(new_n192), .o1(new_n193));
  nona23aa1n09x5               g098(.a(new_n185), .b(new_n179), .c(new_n178), .d(new_n184), .out0(new_n194));
  nor043aa1n06x5               g099(.a(new_n194), .b(new_n173), .c(new_n164), .o1(new_n195));
  nand22aa1n03x5               g100(.a(new_n145), .b(new_n195), .o1(new_n196));
  oai012aa1n02x5               g101(.a(new_n185), .b(new_n184), .c(new_n178), .o1(new_n197));
  oaih12aa1n02x5               g102(.a(new_n197), .b(new_n194), .c(new_n176), .o1(new_n198));
  aoi012aa1n12x5               g103(.a(new_n198), .b(new_n159), .c(new_n195), .o1(new_n199));
  aoai13aa1n12x5               g104(.a(new_n199), .b(new_n196), .c(new_n155), .d(new_n121), .o1(new_n200));
  tech160nm_fixorc02aa1n04x5   g105(.a(\a[17] ), .b(\b[16] ), .out0(new_n201));
  nanb02aa1n02x5               g106(.a(new_n201), .b(new_n197), .out0(new_n202));
  aoi122aa1n02x5               g107(.a(new_n202), .b(new_n177), .c(new_n190), .d(new_n159), .e(new_n195), .o1(new_n203));
  aoi022aa1n02x5               g108(.a(new_n200), .b(new_n201), .c(new_n193), .d(new_n203), .o1(\s[17] ));
  inv000aa1d42x5               g109(.a(\a[17] ), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(\b[16] ), .b(new_n205), .out0(new_n206));
  aobi12aa1n06x5               g111(.a(new_n197), .b(new_n190), .c(new_n177), .out0(new_n207));
  aoai13aa1n02x5               g112(.a(new_n207), .b(new_n191), .c(new_n166), .d(new_n158), .o1(new_n208));
  aoai13aa1n02x5               g113(.a(new_n201), .b(new_n208), .c(new_n122), .d(new_n192), .o1(new_n209));
  xorc02aa1n03x5               g114(.a(\a[18] ), .b(\b[17] ), .out0(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n209), .c(new_n206), .out0(\s[18] ));
  inv000aa1d42x5               g116(.a(\a[18] ), .o1(new_n212));
  xroi22aa1d04x5               g117(.a(new_n205), .b(\b[16] ), .c(new_n212), .d(\b[17] ), .out0(new_n213));
  oaoi03aa1n02x5               g118(.a(\a[18] ), .b(\b[17] ), .c(new_n206), .o1(new_n214));
  nor002aa1d32x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  nand02aa1d08x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  norb02aa1n12x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  aoai13aa1n04x5               g122(.a(new_n217), .b(new_n214), .c(new_n200), .d(new_n213), .o1(new_n218));
  aoi112aa1n02x5               g123(.a(new_n217), .b(new_n214), .c(new_n200), .d(new_n213), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n218), .b(new_n219), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n12x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  nand02aa1d28x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  norb02aa1n12x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  aoib12aa1n02x5               g129(.a(new_n215), .b(new_n223), .c(new_n222), .out0(new_n225));
  nanb02aa1n03x5               g130(.a(new_n215), .b(new_n218), .out0(new_n226));
  aoi022aa1n02x5               g131(.a(new_n226), .b(new_n224), .c(new_n218), .d(new_n225), .o1(\s[20] ));
  nano23aa1n03x7               g132(.a(new_n215), .b(new_n222), .c(new_n223), .d(new_n216), .out0(new_n228));
  nand23aa1n06x5               g133(.a(new_n228), .b(new_n201), .c(new_n210), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  nor042aa1n02x5               g135(.a(\b[17] ), .b(\a[18] ), .o1(new_n231));
  aoi112aa1n09x5               g136(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n232));
  oai112aa1n06x5               g137(.a(new_n217), .b(new_n224), .c(new_n232), .d(new_n231), .o1(new_n233));
  tech160nm_fioai012aa1n03p5x5 g138(.a(new_n223), .b(new_n222), .c(new_n215), .o1(new_n234));
  nanp02aa1n02x5               g139(.a(new_n233), .b(new_n234), .o1(new_n235));
  xorc02aa1n12x5               g140(.a(\a[21] ), .b(\b[20] ), .out0(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n235), .c(new_n200), .d(new_n230), .o1(new_n237));
  nano22aa1n02x4               g142(.a(new_n236), .b(new_n233), .c(new_n234), .out0(new_n238));
  aobi12aa1n02x5               g143(.a(new_n238), .b(new_n200), .c(new_n230), .out0(new_n239));
  norb02aa1n02x5               g144(.a(new_n237), .b(new_n239), .out0(\s[21] ));
  tech160nm_fixorc02aa1n02p5x5 g145(.a(\a[22] ), .b(\b[21] ), .out0(new_n241));
  norp02aa1n02x5               g146(.a(\b[20] ), .b(\a[21] ), .o1(new_n242));
  norp02aa1n02x5               g147(.a(new_n241), .b(new_n242), .o1(new_n243));
  inv000aa1d42x5               g148(.a(\a[21] ), .o1(new_n244));
  oaib12aa1n03x5               g149(.a(new_n237), .b(\b[20] ), .c(new_n244), .out0(new_n245));
  aoi022aa1n02x5               g150(.a(new_n245), .b(new_n241), .c(new_n237), .d(new_n243), .o1(\s[22] ));
  nand02aa1d04x5               g151(.a(new_n241), .b(new_n236), .o1(new_n247));
  nona22aa1n06x5               g152(.a(new_n200), .b(new_n229), .c(new_n247), .out0(new_n248));
  inv000aa1d42x5               g153(.a(\a[22] ), .o1(new_n249));
  oai022aa1n02x5               g154(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n250));
  oaib12aa1n06x5               g155(.a(new_n250), .b(new_n249), .c(\b[21] ), .out0(new_n251));
  aoai13aa1n12x5               g156(.a(new_n251), .b(new_n247), .c(new_n233), .d(new_n234), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  xorc02aa1n12x5               g158(.a(\a[23] ), .b(\b[22] ), .out0(new_n254));
  aob012aa1n03x5               g159(.a(new_n254), .b(new_n248), .c(new_n253), .out0(new_n255));
  xroi22aa1d04x5               g160(.a(new_n244), .b(\b[20] ), .c(new_n249), .d(\b[21] ), .out0(new_n256));
  nanb02aa1n02x5               g161(.a(new_n254), .b(new_n251), .out0(new_n257));
  aoi012aa1n02x5               g162(.a(new_n257), .b(new_n235), .c(new_n256), .o1(new_n258));
  aobi12aa1n02x7               g163(.a(new_n255), .b(new_n258), .c(new_n248), .out0(\s[23] ));
  xorc02aa1n02x5               g164(.a(\a[24] ), .b(\b[23] ), .out0(new_n260));
  nor042aa1d18x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  norp02aa1n02x5               g166(.a(new_n260), .b(new_n261), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n261), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n254), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n263), .b(new_n264), .c(new_n248), .d(new_n253), .o1(new_n265));
  aoi022aa1n03x5               g170(.a(new_n265), .b(new_n260), .c(new_n255), .d(new_n262), .o1(\s[24] ));
  nanp02aa1n02x5               g171(.a(\b[22] ), .b(\a[23] ), .o1(new_n267));
  xnrc02aa1n02x5               g172(.a(\b[23] ), .b(\a[24] ), .out0(new_n268));
  nano22aa1n03x7               g173(.a(new_n268), .b(new_n263), .c(new_n267), .out0(new_n269));
  nano22aa1n02x5               g174(.a(new_n229), .b(new_n256), .c(new_n269), .out0(new_n270));
  oaoi03aa1n02x5               g175(.a(\a[24] ), .b(\b[23] ), .c(new_n263), .o1(new_n271));
  tech160nm_fiao0012aa1n02p5x5 g176(.a(new_n271), .b(new_n252), .c(new_n269), .o(new_n272));
  tech160nm_fixorc02aa1n03p5x5 g177(.a(\a[25] ), .b(\b[24] ), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n272), .c(new_n200), .d(new_n270), .o1(new_n274));
  aoi112aa1n02x5               g179(.a(new_n273), .b(new_n271), .c(new_n252), .d(new_n269), .o1(new_n275));
  aobi12aa1n02x5               g180(.a(new_n275), .b(new_n270), .c(new_n200), .out0(new_n276));
  norb02aa1n02x5               g181(.a(new_n274), .b(new_n276), .out0(\s[25] ));
  xorc02aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .out0(new_n278));
  inv000aa1d42x5               g183(.a(\a[25] ), .o1(new_n279));
  aoib12aa1n02x5               g184(.a(new_n278), .b(new_n279), .c(\b[24] ), .out0(new_n280));
  oaib12aa1n06x5               g185(.a(new_n274), .b(\b[24] ), .c(new_n279), .out0(new_n281));
  aoi022aa1n02x5               g186(.a(new_n281), .b(new_n278), .c(new_n274), .d(new_n280), .o1(\s[26] ));
  and002aa1n02x5               g187(.a(new_n278), .b(new_n273), .o(new_n283));
  aoai13aa1n12x5               g188(.a(new_n283), .b(new_n271), .c(new_n252), .d(new_n269), .o1(new_n284));
  nano32aa1n06x5               g189(.a(new_n229), .b(new_n283), .c(new_n256), .d(new_n269), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n285), .b(new_n208), .c(new_n122), .d(new_n192), .o1(new_n286));
  nanp02aa1n02x5               g191(.a(\b[25] ), .b(\a[26] ), .o1(new_n287));
  oai022aa1n02x5               g192(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(new_n288), .b(new_n287), .o1(new_n289));
  nanp03aa1d12x5               g194(.a(new_n286), .b(new_n284), .c(new_n289), .o1(new_n290));
  xorc02aa1n12x5               g195(.a(\a[27] ), .b(\b[26] ), .out0(new_n291));
  nano32aa1n02x4               g196(.a(new_n291), .b(new_n286), .c(new_n284), .d(new_n289), .out0(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(\s[27] ));
  nanp02aa1n03x5               g198(.a(new_n290), .b(new_n291), .o1(new_n294));
  xorc02aa1n02x5               g199(.a(\a[28] ), .b(\b[27] ), .out0(new_n295));
  norp02aa1n02x5               g200(.a(\b[26] ), .b(\a[27] ), .o1(new_n296));
  norp02aa1n02x5               g201(.a(new_n295), .b(new_n296), .o1(new_n297));
  aoi022aa1n06x5               g202(.a(new_n200), .b(new_n285), .c(new_n287), .d(new_n288), .o1(new_n298));
  inv000aa1n03x5               g203(.a(new_n296), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n291), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n299), .b(new_n300), .c(new_n298), .d(new_n284), .o1(new_n301));
  aoi022aa1n03x5               g206(.a(new_n301), .b(new_n295), .c(new_n294), .d(new_n297), .o1(\s[28] ));
  and002aa1n02x5               g207(.a(new_n295), .b(new_n291), .o(new_n303));
  nanp02aa1n03x5               g208(.a(new_n290), .b(new_n303), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .out0(new_n305));
  oao003aa1n02x5               g210(.a(\a[28] ), .b(\b[27] ), .c(new_n299), .carry(new_n306));
  norb02aa1n02x5               g211(.a(new_n306), .b(new_n305), .out0(new_n307));
  inv000aa1d42x5               g212(.a(new_n303), .o1(new_n308));
  aoai13aa1n03x5               g213(.a(new_n306), .b(new_n308), .c(new_n298), .d(new_n284), .o1(new_n309));
  aoi022aa1n03x5               g214(.a(new_n309), .b(new_n305), .c(new_n304), .d(new_n307), .o1(\s[29] ));
  nanp02aa1n02x5               g215(.a(\b[0] ), .b(\a[1] ), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g217(.a(new_n300), .b(new_n295), .c(new_n305), .out0(new_n313));
  inv000aa1n02x5               g218(.a(new_n313), .o1(new_n314));
  aoi013aa1n02x4               g219(.a(new_n314), .b(new_n286), .c(new_n284), .d(new_n289), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .out0(new_n316));
  oab012aa1n02x4               g221(.a(new_n316), .b(\a[29] ), .c(\b[28] ), .out0(new_n317));
  aoai13aa1n02x5               g222(.a(new_n317), .b(new_n306), .c(\a[29] ), .d(\b[28] ), .o1(new_n318));
  aoi012aa1n03x5               g223(.a(new_n318), .b(new_n290), .c(new_n313), .o1(new_n319));
  tech160nm_fioaoi03aa1n03p5x5 g224(.a(\a[29] ), .b(\b[28] ), .c(new_n306), .o1(new_n320));
  oaoi13aa1n02x7               g225(.a(new_n319), .b(new_n316), .c(new_n320), .d(new_n315), .o1(\s[30] ));
  nano32aa1n06x5               g226(.a(new_n300), .b(new_n316), .c(new_n295), .d(new_n305), .out0(new_n322));
  inv000aa1d42x5               g227(.a(new_n322), .o1(new_n323));
  aoi013aa1n02x4               g228(.a(new_n323), .b(new_n286), .c(new_n284), .d(new_n289), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[31] ), .b(\b[30] ), .out0(new_n325));
  inv000aa1d42x5               g230(.a(\b[29] ), .o1(new_n326));
  oaib12aa1n02x5               g231(.a(new_n320), .b(new_n326), .c(\a[30] ), .out0(new_n327));
  inv000aa1d42x5               g232(.a(\a[30] ), .o1(new_n328));
  aoi012aa1n02x5               g233(.a(new_n325), .b(new_n328), .c(new_n326), .o1(new_n329));
  nanp02aa1n02x5               g234(.a(new_n327), .b(new_n329), .o1(new_n330));
  aoi012aa1n03x5               g235(.a(new_n330), .b(new_n290), .c(new_n322), .o1(new_n331));
  oaib12aa1n02x5               g236(.a(new_n327), .b(\b[29] ), .c(new_n328), .out0(new_n332));
  oaoi13aa1n02x7               g237(.a(new_n331), .b(new_n325), .c(new_n332), .d(new_n324), .o1(\s[31] ));
  norb02aa1n02x5               g238(.a(new_n99), .b(new_n98), .out0(new_n334));
  xobna2aa1n03x5               g239(.a(new_n334), .b(new_n101), .c(new_n97), .out0(\s[3] ));
  obai22aa1n02x7               g240(.a(new_n101), .b(new_n147), .c(\a[3] ), .d(\b[2] ), .out0(new_n336));
  aoi113aa1n02x5               g241(.a(new_n98), .b(new_n104), .c(new_n101), .d(new_n99), .e(new_n97), .o1(new_n337));
  aoi012aa1n02x5               g242(.a(new_n337), .b(new_n336), .c(new_n104), .o1(\s[4] ));
  norb02aa1n02x5               g243(.a(new_n110), .b(new_n109), .out0(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n339), .b(new_n105), .c(new_n106), .out0(\s[5] ));
  norb02aa1n02x5               g245(.a(new_n107), .b(new_n108), .out0(new_n341));
  nanp02aa1n02x5               g246(.a(new_n150), .b(new_n339), .o1(new_n342));
  xnbna2aa1n03x5               g247(.a(new_n341), .b(new_n342), .c(new_n118), .out0(\s[6] ));
  aoai13aa1n02x5               g248(.a(new_n153), .b(new_n119), .c(new_n150), .d(new_n111), .o1(new_n344));
  aoi112aa1n02x5               g249(.a(new_n119), .b(new_n153), .c(new_n150), .d(new_n111), .o1(new_n345));
  norb02aa1n02x5               g250(.a(new_n344), .b(new_n345), .out0(\s[7] ));
  orn002aa1n02x5               g251(.a(\a[7] ), .b(\b[6] ), .o(new_n347));
  xnbna2aa1n03x5               g252(.a(new_n152), .b(new_n344), .c(new_n347), .out0(\s[8] ));
  aoi112aa1n02x5               g253(.a(new_n123), .b(new_n120), .c(new_n116), .d(new_n119), .o1(new_n349));
  aoi022aa1n02x5               g254(.a(new_n122), .b(new_n123), .c(new_n155), .d(new_n349), .o1(\s[9] ));
endmodule


