// Benchmark "adder" written by ABC on Wed Jul 17 18:43:22 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n336, new_n339, new_n341,
    new_n342, new_n343, new_n345;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n12x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nor002aa1n20x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  nand22aa1n04x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nona23aa1n06x5               g005(.a(new_n100), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n101));
  norp02aa1n04x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  nanb02aa1n02x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  xnrc02aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .out0(new_n105));
  nor043aa1n03x5               g010(.a(new_n101), .b(new_n104), .c(new_n105), .o1(new_n106));
  inv000aa1d42x5               g011(.a(\a[2] ), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\b[1] ), .o1(new_n108));
  nand42aa1n02x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  oaoi03aa1n02x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  nor002aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nor022aa1n04x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nona23aa1n02x4               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  tech160nm_fiao0012aa1n02p5x5 g020(.a(new_n111), .b(new_n113), .c(new_n112), .o(new_n116));
  oabi12aa1n06x5               g021(.a(new_n116), .b(new_n115), .c(new_n110), .out0(new_n117));
  inv000aa1d42x5               g022(.a(new_n97), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(new_n99), .b(new_n98), .o1(new_n119));
  norp02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n103), .b(new_n120), .c(new_n102), .o1(new_n121));
  oai112aa1n04x5               g026(.a(new_n118), .b(new_n119), .c(new_n101), .d(new_n121), .o1(new_n122));
  aoi012aa1n02x5               g027(.a(new_n122), .b(new_n117), .c(new_n106), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[9] ), .b(\b[8] ), .c(new_n123), .o1(new_n124));
  xorb03aa1n02x5               g029(.a(new_n124), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d32x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  nand22aa1n04x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  nor002aa1d24x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nand42aa1n02x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nor002aa1n16x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand02aa1n08x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nano23aa1n06x5               g037(.a(new_n129), .b(new_n131), .c(new_n132), .d(new_n130), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n122), .c(new_n117), .d(new_n106), .o1(new_n134));
  oai012aa1d24x5               g039(.a(new_n132), .b(new_n131), .c(new_n129), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n128), .b(new_n134), .c(new_n135), .out0(\s[11] ));
  inv000aa1d42x5               g041(.a(new_n126), .o1(new_n137));
  inv000aa1n03x5               g042(.a(new_n123), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n135), .o1(new_n139));
  aoai13aa1n02x5               g044(.a(new_n128), .b(new_n139), .c(new_n138), .d(new_n133), .o1(new_n140));
  nor002aa1d32x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1n04x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n140), .c(new_n137), .out0(\s[12] ));
  nor002aa1d24x5               g049(.a(\b[12] ), .b(\a[13] ), .o1(new_n145));
  nand42aa1n02x5               g050(.a(\b[12] ), .b(\a[13] ), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n141), .o1(new_n148));
  nand42aa1n02x5               g053(.a(new_n126), .b(new_n142), .o1(new_n149));
  nona23aa1d18x5               g054(.a(new_n142), .b(new_n127), .c(new_n126), .d(new_n141), .out0(new_n150));
  oai112aa1n06x5               g055(.a(new_n149), .b(new_n148), .c(new_n150), .d(new_n135), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  nona23aa1n02x4               g057(.a(new_n132), .b(new_n130), .c(new_n129), .d(new_n131), .out0(new_n153));
  norp02aa1n02x5               g058(.a(new_n150), .b(new_n153), .o1(new_n154));
  aoai13aa1n03x5               g059(.a(new_n154), .b(new_n122), .c(new_n117), .d(new_n106), .o1(new_n155));
  xnbna2aa1n03x5               g060(.a(new_n147), .b(new_n155), .c(new_n152), .out0(\s[13] ));
  inv000aa1d42x5               g061(.a(new_n145), .o1(new_n157));
  aob012aa1n02x5               g062(.a(new_n147), .b(new_n155), .c(new_n152), .out0(new_n158));
  norp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n03x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n158), .c(new_n157), .out0(\s[14] ));
  nona23aa1d18x5               g067(.a(new_n160), .b(new_n146), .c(new_n145), .d(new_n159), .out0(new_n163));
  oai012aa1n02x5               g068(.a(new_n160), .b(new_n159), .c(new_n145), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n155), .d(new_n152), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand42aa1n08x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  nor002aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1n03x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norb02aa1n02x7               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  aoi112aa1n02x5               g077(.a(new_n172), .b(new_n167), .c(new_n165), .d(new_n169), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n172), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(\s[16] ));
  nano23aa1n03x7               g080(.a(new_n167), .b(new_n170), .c(new_n171), .d(new_n168), .out0(new_n176));
  nano23aa1n02x4               g081(.a(new_n163), .b(new_n150), .c(new_n176), .d(new_n133), .out0(new_n177));
  aoai13aa1n06x5               g082(.a(new_n177), .b(new_n122), .c(new_n106), .d(new_n117), .o1(new_n178));
  nano22aa1n03x7               g083(.a(new_n163), .b(new_n169), .c(new_n172), .out0(new_n179));
  aoi112aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n180));
  obai22aa1n02x7               g085(.a(new_n176), .b(new_n164), .c(\a[16] ), .d(\b[15] ), .out0(new_n181));
  aoi112aa1n09x5               g086(.a(new_n181), .b(new_n180), .c(new_n151), .d(new_n179), .o1(new_n182));
  xorc02aa1n02x5               g087(.a(\a[17] ), .b(\b[16] ), .out0(new_n183));
  xnbna2aa1n03x5               g088(.a(new_n183), .b(new_n182), .c(new_n178), .out0(\s[17] ));
  inv000aa1d42x5               g089(.a(\a[17] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\b[16] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(new_n186), .b(new_n185), .o1(new_n187));
  nano23aa1n02x4               g092(.a(new_n97), .b(new_n99), .c(new_n100), .d(new_n98), .out0(new_n188));
  nona22aa1n02x4               g093(.a(new_n188), .b(new_n105), .c(new_n104), .out0(new_n189));
  oao003aa1n02x5               g094(.a(new_n107), .b(new_n108), .c(new_n109), .carry(new_n190));
  nano23aa1n02x4               g095(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n191));
  aoi012aa1n02x5               g096(.a(new_n116), .b(new_n191), .c(new_n190), .o1(new_n192));
  nanb02aa1n02x5               g097(.a(new_n97), .b(new_n98), .out0(new_n193));
  nanb02aa1d24x5               g098(.a(new_n99), .b(new_n100), .out0(new_n194));
  norp03aa1n02x5               g099(.a(new_n121), .b(new_n194), .c(new_n193), .o1(new_n195));
  nano22aa1n03x7               g100(.a(new_n195), .b(new_n118), .c(new_n119), .out0(new_n196));
  nanp02aa1n02x5               g101(.a(new_n179), .b(new_n154), .o1(new_n197));
  oaoi13aa1n09x5               g102(.a(new_n197), .b(new_n196), .c(new_n192), .d(new_n189), .o1(new_n198));
  nanp02aa1n04x5               g103(.a(new_n151), .b(new_n179), .o1(new_n199));
  nano22aa1n02x4               g104(.a(new_n164), .b(new_n169), .c(new_n172), .out0(new_n200));
  nona32aa1n02x4               g105(.a(new_n199), .b(new_n200), .c(new_n180), .d(new_n170), .out0(new_n201));
  oai012aa1n02x5               g106(.a(new_n183), .b(new_n201), .c(new_n198), .o1(new_n202));
  nor002aa1n02x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nanb02aa1n09x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  xobna2aa1n03x5               g110(.a(new_n205), .b(new_n202), .c(new_n187), .out0(\s[18] ));
  nanp02aa1n02x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  nano22aa1n12x5               g112(.a(new_n205), .b(new_n187), .c(new_n207), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoai13aa1n04x5               g114(.a(new_n204), .b(new_n203), .c(new_n185), .d(new_n186), .o1(new_n210));
  aoai13aa1n06x5               g115(.a(new_n210), .b(new_n209), .c(new_n182), .d(new_n178), .o1(new_n211));
  xorb03aa1n03x5               g116(.a(new_n211), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n12x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nanp02aa1n04x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  nanb02aa1d24x5               g120(.a(new_n214), .b(new_n215), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  inv040aa1d32x5               g122(.a(\a[20] ), .o1(new_n218));
  inv000aa1d42x5               g123(.a(\b[19] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(new_n219), .b(new_n218), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nand02aa1d06x5               g126(.a(new_n220), .b(new_n221), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoi112aa1n02x7               g128(.a(new_n214), .b(new_n223), .c(new_n211), .d(new_n217), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n214), .o1(new_n225));
  tech160nm_finand02aa1n03p5x5 g130(.a(new_n211), .b(new_n217), .o1(new_n226));
  aoi012aa1n03x5               g131(.a(new_n222), .b(new_n226), .c(new_n225), .o1(new_n227));
  norp02aa1n03x5               g132(.a(new_n227), .b(new_n224), .o1(\s[20] ));
  nona22aa1n02x4               g133(.a(new_n208), .b(new_n216), .c(new_n222), .out0(new_n229));
  nanp02aa1n02x5               g134(.a(new_n214), .b(new_n221), .o1(new_n230));
  nor003aa1n08x5               g135(.a(new_n210), .b(new_n216), .c(new_n222), .o1(new_n231));
  nano22aa1n12x5               g136(.a(new_n231), .b(new_n220), .c(new_n230), .out0(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n229), .c(new_n182), .d(new_n178), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  xorc02aa1n12x5               g140(.a(\a[21] ), .b(\b[20] ), .out0(new_n236));
  xorc02aa1n12x5               g141(.a(\a[22] ), .b(\b[21] ), .out0(new_n237));
  aoi112aa1n02x7               g142(.a(new_n235), .b(new_n237), .c(new_n233), .d(new_n236), .o1(new_n238));
  inv000aa1n02x5               g143(.a(new_n235), .o1(new_n239));
  tech160nm_finand02aa1n03p5x5 g144(.a(new_n233), .b(new_n236), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n237), .o1(new_n241));
  aoi012aa1n03x5               g146(.a(new_n241), .b(new_n240), .c(new_n239), .o1(new_n242));
  norp02aa1n03x5               g147(.a(new_n242), .b(new_n238), .o1(\s[22] ));
  norp02aa1n02x5               g148(.a(\b[19] ), .b(\a[20] ), .o1(new_n244));
  nona23aa1n02x4               g149(.a(new_n221), .b(new_n215), .c(new_n214), .d(new_n244), .out0(new_n245));
  oai112aa1n02x5               g150(.a(new_n230), .b(new_n220), .c(new_n245), .d(new_n210), .o1(new_n246));
  and002aa1n02x5               g151(.a(new_n237), .b(new_n236), .o(new_n247));
  oaoi03aa1n02x5               g152(.a(\a[22] ), .b(\b[21] ), .c(new_n239), .o1(new_n248));
  aoi012aa1n02x5               g153(.a(new_n248), .b(new_n246), .c(new_n247), .o1(new_n249));
  nona23aa1n03x5               g154(.a(new_n236), .b(new_n208), .c(new_n241), .d(new_n245), .out0(new_n250));
  aoai13aa1n06x5               g155(.a(new_n249), .b(new_n250), .c(new_n182), .d(new_n178), .o1(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g157(.a(\b[22] ), .b(\a[23] ), .o1(new_n253));
  nand22aa1n02x5               g158(.a(\b[22] ), .b(\a[23] ), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n254), .b(new_n253), .out0(new_n255));
  nor042aa1n03x5               g160(.a(\b[23] ), .b(\a[24] ), .o1(new_n256));
  nand42aa1n03x5               g161(.a(\b[23] ), .b(\a[24] ), .o1(new_n257));
  norb02aa1n02x5               g162(.a(new_n257), .b(new_n256), .out0(new_n258));
  aoi112aa1n02x7               g163(.a(new_n253), .b(new_n258), .c(new_n251), .d(new_n255), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n253), .o1(new_n260));
  nand42aa1n02x5               g165(.a(new_n251), .b(new_n255), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n258), .o1(new_n262));
  aoi012aa1n03x5               g167(.a(new_n262), .b(new_n261), .c(new_n260), .o1(new_n263));
  nor002aa1n02x5               g168(.a(new_n263), .b(new_n259), .o1(\s[24] ));
  nano23aa1d15x5               g169(.a(new_n253), .b(new_n256), .c(new_n257), .d(new_n254), .out0(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  nona23aa1n02x4               g171(.a(new_n247), .b(new_n208), .c(new_n266), .d(new_n245), .out0(new_n267));
  aoi112aa1n02x5               g172(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n268));
  aoi112aa1n09x5               g173(.a(new_n268), .b(new_n256), .c(new_n265), .d(new_n248), .o1(new_n269));
  nand23aa1d12x5               g174(.a(new_n265), .b(new_n236), .c(new_n237), .o1(new_n270));
  oai012aa1d24x5               g175(.a(new_n269), .b(new_n232), .c(new_n270), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n267), .c(new_n182), .d(new_n178), .o1(new_n273));
  xorb03aa1n02x5               g178(.a(new_n273), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g179(.a(\b[24] ), .b(\a[25] ), .o1(new_n275));
  xorc02aa1n02x5               g180(.a(\a[25] ), .b(\b[24] ), .out0(new_n276));
  tech160nm_fixorc02aa1n03p5x5 g181(.a(\a[26] ), .b(\b[25] ), .out0(new_n277));
  aoi112aa1n03x4               g182(.a(new_n275), .b(new_n277), .c(new_n273), .d(new_n276), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n275), .o1(new_n279));
  nand42aa1n02x5               g184(.a(new_n273), .b(new_n276), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n277), .o1(new_n281));
  aoi012aa1n03x5               g186(.a(new_n281), .b(new_n280), .c(new_n279), .o1(new_n282));
  norp02aa1n03x5               g187(.a(new_n282), .b(new_n278), .o1(\s[26] ));
  and002aa1n02x5               g188(.a(new_n277), .b(new_n276), .o(new_n284));
  nano22aa1n03x7               g189(.a(new_n250), .b(new_n265), .c(new_n284), .out0(new_n285));
  oai012aa1n06x5               g190(.a(new_n285), .b(new_n201), .c(new_n198), .o1(new_n286));
  norp02aa1n02x5               g191(.a(\b[25] ), .b(\a[26] ), .o1(new_n287));
  aoi112aa1n02x5               g192(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n288));
  aoi112aa1n06x5               g193(.a(new_n287), .b(new_n288), .c(new_n271), .d(new_n284), .o1(new_n289));
  xorc02aa1n12x5               g194(.a(\a[27] ), .b(\b[26] ), .out0(new_n290));
  xnbna2aa1n03x5               g195(.a(new_n290), .b(new_n286), .c(new_n289), .out0(\s[27] ));
  nor042aa1n03x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n290), .o1(new_n294));
  aoi012aa1n02x7               g199(.a(new_n294), .b(new_n286), .c(new_n289), .o1(new_n295));
  tech160nm_fixnrc02aa1n04x5   g200(.a(\b[27] ), .b(\a[28] ), .out0(new_n296));
  nano22aa1n02x4               g201(.a(new_n295), .b(new_n293), .c(new_n296), .out0(new_n297));
  nand02aa1d04x5               g202(.a(new_n182), .b(new_n178), .o1(new_n298));
  nanp02aa1n02x5               g203(.a(new_n265), .b(new_n248), .o1(new_n299));
  nona22aa1n02x4               g204(.a(new_n299), .b(new_n268), .c(new_n256), .out0(new_n300));
  inv040aa1n03x5               g205(.a(new_n270), .o1(new_n301));
  aoai13aa1n02x5               g206(.a(new_n284), .b(new_n300), .c(new_n246), .d(new_n301), .o1(new_n302));
  nona22aa1n03x5               g207(.a(new_n302), .b(new_n288), .c(new_n287), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n290), .b(new_n303), .c(new_n298), .d(new_n285), .o1(new_n304));
  aoi012aa1n03x5               g209(.a(new_n296), .b(new_n304), .c(new_n293), .o1(new_n305));
  norp02aa1n03x5               g210(.a(new_n305), .b(new_n297), .o1(\s[28] ));
  norb02aa1d21x5               g211(.a(new_n290), .b(new_n296), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n303), .c(new_n298), .d(new_n285), .o1(new_n308));
  oao003aa1n02x5               g213(.a(\a[28] ), .b(\b[27] ), .c(new_n293), .carry(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[28] ), .b(\a[29] ), .out0(new_n310));
  aoi012aa1n03x5               g215(.a(new_n310), .b(new_n308), .c(new_n309), .o1(new_n311));
  inv000aa1d42x5               g216(.a(new_n307), .o1(new_n312));
  tech160nm_fiaoi012aa1n02p5x5 g217(.a(new_n312), .b(new_n286), .c(new_n289), .o1(new_n313));
  nano22aa1n02x4               g218(.a(new_n313), .b(new_n309), .c(new_n310), .out0(new_n314));
  nor002aa1n02x5               g219(.a(new_n311), .b(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1d15x5               g221(.a(new_n290), .b(new_n310), .c(new_n296), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n303), .c(new_n298), .d(new_n285), .o1(new_n318));
  oao003aa1n02x5               g223(.a(\a[29] ), .b(\b[28] ), .c(new_n309), .carry(new_n319));
  xnrc02aa1n02x5               g224(.a(\b[29] ), .b(\a[30] ), .out0(new_n320));
  aoi012aa1n03x5               g225(.a(new_n320), .b(new_n318), .c(new_n319), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n317), .o1(new_n322));
  tech160nm_fiaoi012aa1n02p5x5 g227(.a(new_n322), .b(new_n286), .c(new_n289), .o1(new_n323));
  nano22aa1n03x5               g228(.a(new_n323), .b(new_n319), .c(new_n320), .out0(new_n324));
  norp02aa1n03x5               g229(.a(new_n321), .b(new_n324), .o1(\s[30] ));
  norb02aa1n06x5               g230(.a(new_n317), .b(new_n320), .out0(new_n326));
  inv000aa1d42x5               g231(.a(new_n326), .o1(new_n327));
  tech160nm_fiaoi012aa1n02p5x5 g232(.a(new_n327), .b(new_n286), .c(new_n289), .o1(new_n328));
  oao003aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .c(new_n319), .carry(new_n329));
  xnrc02aa1n02x5               g234(.a(\b[30] ), .b(\a[31] ), .out0(new_n330));
  nano22aa1n03x5               g235(.a(new_n328), .b(new_n329), .c(new_n330), .out0(new_n331));
  aoai13aa1n03x5               g236(.a(new_n326), .b(new_n303), .c(new_n298), .d(new_n285), .o1(new_n332));
  tech160nm_fiaoi012aa1n02p5x5 g237(.a(new_n330), .b(new_n332), .c(new_n329), .o1(new_n333));
  nor002aa1n02x5               g238(.a(new_n333), .b(new_n331), .o1(\s[31] ));
  xnrb03aa1n02x5               g239(.a(new_n110), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g240(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n336));
  xorb03aa1n02x5               g241(.a(new_n336), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g242(.a(new_n117), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g243(.a(\a[5] ), .b(\b[4] ), .c(new_n192), .o1(new_n339));
  xorb03aa1n02x5               g244(.a(new_n339), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g245(.a(new_n194), .o1(new_n341));
  aoai13aa1n02x5               g246(.a(new_n341), .b(new_n102), .c(new_n339), .d(new_n103), .o1(new_n342));
  aoi112aa1n02x5               g247(.a(new_n341), .b(new_n102), .c(new_n339), .d(new_n103), .o1(new_n343));
  norb02aa1n02x5               g248(.a(new_n342), .b(new_n343), .out0(\s[7] ));
  inv000aa1d42x5               g249(.a(new_n99), .o1(new_n345));
  xobna2aa1n03x5               g250(.a(new_n193), .b(new_n342), .c(new_n345), .out0(\s[8] ));
  xnrb03aa1n02x5               g251(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


