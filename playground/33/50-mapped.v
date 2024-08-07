// Benchmark "adder" written by ABC on Thu Jul 18 05:20:22 2024

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
    new_n118, new_n119, new_n121, new_n122, new_n123, new_n124, new_n125,
    new_n126, new_n128, new_n129, new_n130, new_n131, new_n133, new_n134,
    new_n135, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n143, new_n144, new_n145, new_n146, new_n148, new_n149, new_n150,
    new_n151, new_n152, new_n153, new_n155, new_n156, new_n157, new_n158,
    new_n159, new_n160, new_n161, new_n163, new_n164, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n179, new_n180, new_n181,
    new_n182, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n313, new_n315, new_n316, new_n317,
    new_n318, new_n320, new_n322, new_n324, new_n325, new_n326, new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1d10x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nand02aa1d16x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1n12x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  xnrc02aa1n12x5               g005(.a(\b[2] ), .b(\a[3] ), .out0(new_n101));
  oa0022aa1n06x5               g006(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n102));
  oai012aa1d24x5               g007(.a(new_n102), .b(new_n101), .c(new_n100), .o1(new_n103));
  nor042aa1n04x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nanp02aa1n09x5               g009(.a(\b[7] ), .b(\a[8] ), .o1(new_n105));
  nor042aa1n04x5               g010(.a(\b[7] ), .b(\a[8] ), .o1(new_n106));
  norb03aa1d15x5               g011(.a(new_n105), .b(new_n104), .c(new_n106), .out0(new_n107));
  tech160nm_fixorc02aa1n04x5   g012(.a(\a[5] ), .b(\b[4] ), .out0(new_n108));
  nand02aa1d06x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor042aa1n02x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  aoi022aa1n12x5               g015(.a(\b[6] ), .b(\a[7] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n111));
  nanb03aa1n12x5               g016(.a(new_n110), .b(new_n111), .c(new_n109), .out0(new_n112));
  nano22aa1n03x7               g017(.a(new_n112), .b(new_n108), .c(new_n107), .out0(new_n113));
  nor002aa1n03x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nand42aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  aoai13aa1n06x5               g020(.a(new_n115), .b(new_n110), .c(new_n114), .d(new_n109), .o1(new_n116));
  aoi022aa1n06x5               g021(.a(new_n116), .b(new_n107), .c(\a[8] ), .d(\b[7] ), .o1(new_n117));
  aoi012aa1d18x5               g022(.a(new_n117), .b(new_n113), .c(new_n103), .o1(new_n118));
  oaoi03aa1n09x5               g023(.a(\a[9] ), .b(\b[8] ), .c(new_n118), .o1(new_n119));
  xorb03aa1n02x5               g024(.a(new_n119), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanp02aa1n02x5               g025(.a(\b[10] ), .b(\a[11] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\a[11] ), .o1(new_n122));
  nanb02aa1n12x5               g027(.a(\b[10] ), .b(new_n122), .out0(new_n123));
  nor042aa1n04x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nand02aa1d24x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  oai012aa1n09x5               g030(.a(new_n125), .b(new_n119), .c(new_n124), .o1(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n121), .c(new_n123), .out0(\s[11] ));
  nanp02aa1n02x5               g032(.a(new_n123), .b(new_n121), .o1(new_n128));
  xnrc02aa1n03x5               g033(.a(\b[11] ), .b(\a[12] ), .out0(new_n129));
  oaoi13aa1n02x7               g034(.a(new_n129), .b(new_n123), .c(new_n126), .d(new_n128), .o1(new_n130));
  oai112aa1n03x5               g035(.a(new_n123), .b(new_n129), .c(new_n126), .d(new_n128), .o1(new_n131));
  norb02aa1n02x7               g036(.a(new_n131), .b(new_n130), .out0(\s[12] ));
  norb02aa1n06x4               g037(.a(new_n125), .b(new_n124), .out0(new_n133));
  xorc02aa1n12x5               g038(.a(\a[9] ), .b(\b[8] ), .out0(new_n134));
  nona23aa1n02x4               g039(.a(new_n133), .b(new_n134), .c(new_n129), .d(new_n128), .out0(new_n135));
  nanp02aa1n02x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  oaih22aa1d12x5               g041(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n137));
  aoi022aa1n06x5               g042(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n138));
  oai022aa1n02x5               g043(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n139));
  aoai13aa1n06x5               g044(.a(new_n136), .b(new_n139), .c(new_n137), .d(new_n138), .o1(new_n140));
  tech160nm_fioai012aa1n05x5   g045(.a(new_n140), .b(new_n118), .c(new_n135), .o1(new_n141));
  xorb03aa1n02x5               g046(.a(new_n141), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g047(.a(\a[14] ), .o1(new_n143));
  nor042aa1n03x5               g048(.a(\b[12] ), .b(\a[13] ), .o1(new_n144));
  xorc02aa1n06x5               g049(.a(\a[13] ), .b(\b[12] ), .out0(new_n145));
  aoi012aa1n02x5               g050(.a(new_n144), .b(new_n141), .c(new_n145), .o1(new_n146));
  xorb03aa1n02x5               g051(.a(new_n146), .b(\b[13] ), .c(new_n143), .out0(\s[14] ));
  inv000aa1d42x5               g052(.a(\a[15] ), .o1(new_n148));
  inv000aa1d42x5               g053(.a(\a[13] ), .o1(new_n149));
  xroi22aa1d04x5               g054(.a(new_n149), .b(\b[12] ), .c(new_n143), .d(\b[13] ), .out0(new_n150));
  inv000aa1d42x5               g055(.a(\b[13] ), .o1(new_n151));
  tech160nm_fioaoi03aa1n02p5x5 g056(.a(new_n143), .b(new_n151), .c(new_n144), .o1(new_n152));
  aobi12aa1n03x5               g057(.a(new_n152), .b(new_n141), .c(new_n150), .out0(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[14] ), .c(new_n148), .out0(\s[15] ));
  inv000aa1d42x5               g059(.a(\b[14] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(new_n155), .b(new_n148), .o1(new_n156));
  xnrc02aa1n02x5               g061(.a(\b[14] ), .b(\a[15] ), .out0(new_n157));
  xnrc02aa1n02x5               g062(.a(\b[15] ), .b(\a[16] ), .out0(new_n158));
  oaoi13aa1n02x5               g063(.a(new_n158), .b(new_n156), .c(new_n153), .d(new_n157), .o1(new_n159));
  nor002aa1n02x5               g064(.a(new_n153), .b(new_n157), .o1(new_n160));
  nano22aa1n03x5               g065(.a(new_n160), .b(new_n156), .c(new_n158), .out0(new_n161));
  norp02aa1n02x5               g066(.a(new_n159), .b(new_n161), .o1(\s[16] ));
  nanp03aa1n02x5               g067(.a(new_n133), .b(new_n121), .c(new_n123), .o1(new_n163));
  inv000aa1n02x5               g068(.a(new_n134), .o1(new_n164));
  tech160nm_fixorc02aa1n02p5x5 g069(.a(\a[14] ), .b(\b[13] ), .out0(new_n165));
  nanp02aa1n03x5               g070(.a(new_n165), .b(new_n145), .o1(new_n166));
  xorc02aa1n03x5               g071(.a(\a[15] ), .b(\b[14] ), .out0(new_n167));
  tech160nm_fixorc02aa1n02p5x5 g072(.a(\a[16] ), .b(\b[15] ), .out0(new_n168));
  nano22aa1n03x5               g073(.a(new_n166), .b(new_n167), .c(new_n168), .out0(new_n169));
  nona32aa1n03x5               g074(.a(new_n169), .b(new_n163), .c(new_n164), .d(new_n129), .out0(new_n170));
  inv000aa1d42x5               g075(.a(\a[16] ), .o1(new_n171));
  inv000aa1d42x5               g076(.a(\b[15] ), .o1(new_n172));
  nanp02aa1n02x5               g077(.a(new_n172), .b(new_n171), .o1(new_n173));
  oai022aa1n02x5               g078(.a(new_n148), .b(new_n155), .c(new_n172), .d(new_n171), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n173), .b(new_n174), .c(new_n152), .d(new_n156), .o1(new_n175));
  aoib12aa1n06x5               g080(.a(new_n175), .b(new_n169), .c(new_n140), .out0(new_n176));
  oai012aa1n18x5               g081(.a(new_n176), .b(new_n118), .c(new_n170), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g083(.a(\a[18] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\a[17] ), .o1(new_n180));
  inv000aa1d42x5               g085(.a(\b[16] ), .o1(new_n181));
  oaoi03aa1n02x5               g086(.a(new_n180), .b(new_n181), .c(new_n177), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[17] ), .c(new_n179), .out0(\s[18] ));
  xroi22aa1d06x4               g088(.a(new_n180), .b(\b[16] ), .c(new_n179), .d(\b[17] ), .out0(new_n184));
  nor042aa1n02x5               g089(.a(\b[17] ), .b(\a[18] ), .o1(new_n185));
  aoi112aa1n09x5               g090(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n186));
  nor042aa1n06x5               g091(.a(new_n186), .b(new_n185), .o1(new_n187));
  inv000aa1d42x5               g092(.a(new_n187), .o1(new_n188));
  nor022aa1n08x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  nand22aa1n04x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  norb02aa1n06x4               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n188), .c(new_n177), .d(new_n184), .o1(new_n192));
  aoi112aa1n02x5               g097(.a(new_n191), .b(new_n188), .c(new_n177), .d(new_n184), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n192), .b(new_n193), .out0(\s[19] ));
  xnrc02aa1n02x5               g099(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n12x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  nand02aa1n12x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  norb02aa1n09x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  nona22aa1n03x5               g103(.a(new_n192), .b(new_n198), .c(new_n189), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n198), .o1(new_n200));
  oaoi13aa1n06x5               g105(.a(new_n200), .b(new_n192), .c(\a[19] ), .d(\b[18] ), .o1(new_n201));
  norb02aa1n03x4               g106(.a(new_n199), .b(new_n201), .out0(\s[20] ));
  nona23aa1n09x5               g107(.a(new_n197), .b(new_n190), .c(new_n189), .d(new_n196), .out0(new_n203));
  inv040aa1n04x5               g108(.a(new_n203), .o1(new_n204));
  nand22aa1n03x5               g109(.a(new_n184), .b(new_n204), .o1(new_n205));
  inv040aa1n03x5               g110(.a(new_n205), .o1(new_n206));
  oaih12aa1n02x5               g111(.a(new_n197), .b(new_n196), .c(new_n189), .o1(new_n207));
  oai012aa1n04x7               g112(.a(new_n207), .b(new_n203), .c(new_n187), .o1(new_n208));
  xorc02aa1n12x5               g113(.a(\a[21] ), .b(\b[20] ), .out0(new_n209));
  aoai13aa1n06x5               g114(.a(new_n209), .b(new_n208), .c(new_n177), .d(new_n206), .o1(new_n210));
  aoi112aa1n02x5               g115(.a(new_n209), .b(new_n208), .c(new_n177), .d(new_n206), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(\s[21] ));
  nor002aa1n02x5               g117(.a(\b[20] ), .b(\a[21] ), .o1(new_n213));
  xorc02aa1n12x5               g118(.a(\a[22] ), .b(\b[21] ), .out0(new_n214));
  nona22aa1n03x5               g119(.a(new_n210), .b(new_n214), .c(new_n213), .out0(new_n215));
  inv000aa1d42x5               g120(.a(new_n214), .o1(new_n216));
  oaoi13aa1n06x5               g121(.a(new_n216), .b(new_n210), .c(\a[21] ), .d(\b[20] ), .o1(new_n217));
  norb02aa1n03x4               g122(.a(new_n215), .b(new_n217), .out0(\s[22] ));
  nand02aa1d06x5               g123(.a(new_n214), .b(new_n209), .o1(new_n219));
  nano22aa1n02x4               g124(.a(new_n219), .b(new_n184), .c(new_n204), .out0(new_n220));
  oai112aa1n03x5               g125(.a(new_n191), .b(new_n198), .c(new_n186), .d(new_n185), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\a[22] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[21] ), .o1(new_n223));
  oao003aa1n02x5               g128(.a(new_n222), .b(new_n223), .c(new_n213), .carry(new_n224));
  inv000aa1n02x5               g129(.a(new_n224), .o1(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n219), .c(new_n221), .d(new_n207), .o1(new_n226));
  tech160nm_fixorc02aa1n02p5x5 g131(.a(\a[23] ), .b(\b[22] ), .out0(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n226), .c(new_n177), .d(new_n220), .o1(new_n228));
  aoi112aa1n02x5               g133(.a(new_n227), .b(new_n226), .c(new_n177), .d(new_n220), .o1(new_n229));
  norb02aa1n02x5               g134(.a(new_n228), .b(new_n229), .out0(\s[23] ));
  nor002aa1n02x5               g135(.a(\b[22] ), .b(\a[23] ), .o1(new_n231));
  xorc02aa1n12x5               g136(.a(\a[24] ), .b(\b[23] ), .out0(new_n232));
  nona22aa1n02x5               g137(.a(new_n228), .b(new_n232), .c(new_n231), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n232), .o1(new_n234));
  oaoi13aa1n02x7               g139(.a(new_n234), .b(new_n228), .c(\a[23] ), .d(\b[22] ), .o1(new_n235));
  norb02aa1n03x4               g140(.a(new_n233), .b(new_n235), .out0(\s[24] ));
  nand22aa1n03x5               g141(.a(new_n232), .b(new_n227), .o1(new_n237));
  norp03aa1n02x5               g142(.a(new_n205), .b(new_n219), .c(new_n237), .o1(new_n238));
  inv000aa1n03x5               g143(.a(new_n219), .o1(new_n239));
  inv030aa1n02x5               g144(.a(new_n237), .o1(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n224), .c(new_n208), .d(new_n239), .o1(new_n241));
  inv000aa1d42x5               g146(.a(\a[24] ), .o1(new_n242));
  inv000aa1d42x5               g147(.a(\b[23] ), .o1(new_n243));
  oao003aa1n02x5               g148(.a(new_n242), .b(new_n243), .c(new_n231), .carry(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  nanp02aa1n02x5               g150(.a(new_n241), .b(new_n245), .o1(new_n246));
  tech160nm_fixorc02aa1n02p5x5 g151(.a(\a[25] ), .b(\b[24] ), .out0(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n246), .c(new_n177), .d(new_n238), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n247), .b(new_n246), .c(new_n177), .d(new_n238), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n248), .b(new_n249), .out0(\s[25] ));
  xorc02aa1n12x5               g155(.a(\a[26] ), .b(\b[25] ), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  oai112aa1n03x5               g157(.a(new_n248), .b(new_n252), .c(\b[24] ), .d(\a[25] ), .o1(new_n253));
  oaoi13aa1n02x7               g158(.a(new_n252), .b(new_n248), .c(\a[25] ), .d(\b[24] ), .o1(new_n254));
  norb02aa1n03x4               g159(.a(new_n253), .b(new_n254), .out0(\s[26] ));
  inv000aa1d42x5               g160(.a(new_n103), .o1(new_n256));
  nanb03aa1n02x5               g161(.a(new_n112), .b(new_n107), .c(new_n108), .out0(new_n257));
  oabi12aa1n02x5               g162(.a(new_n117), .b(new_n257), .c(new_n256), .out0(new_n258));
  nona22aa1n02x4               g163(.a(new_n150), .b(new_n157), .c(new_n158), .out0(new_n259));
  nor042aa1n02x5               g164(.a(new_n259), .b(new_n135), .o1(new_n260));
  oabi12aa1n02x5               g165(.a(new_n175), .b(new_n259), .c(new_n140), .out0(new_n261));
  nanp02aa1n02x5               g166(.a(new_n251), .b(new_n247), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n262), .o1(new_n263));
  nano32aa1n03x7               g168(.a(new_n205), .b(new_n263), .c(new_n239), .d(new_n240), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n261), .c(new_n258), .d(new_n260), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n263), .b(new_n244), .c(new_n226), .d(new_n240), .o1(new_n266));
  oai022aa1n02x5               g171(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n267));
  aob012aa1n02x5               g172(.a(new_n267), .b(\b[25] ), .c(\a[26] ), .out0(new_n268));
  nor042aa1n03x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  norb02aa1n09x5               g175(.a(new_n270), .b(new_n269), .out0(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  aoi013aa1n03x5               g177(.a(new_n272), .b(new_n265), .c(new_n266), .d(new_n268), .o1(new_n273));
  nona32aa1n03x5               g178(.a(new_n206), .b(new_n262), .c(new_n237), .d(new_n219), .out0(new_n274));
  oaoi13aa1n09x5               g179(.a(new_n274), .b(new_n176), .c(new_n118), .d(new_n170), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n268), .b(new_n262), .c(new_n241), .d(new_n245), .o1(new_n276));
  norp03aa1n02x5               g181(.a(new_n276), .b(new_n275), .c(new_n271), .o1(new_n277));
  norp02aa1n02x5               g182(.a(new_n273), .b(new_n277), .o1(\s[27] ));
  inv000aa1n06x5               g183(.a(new_n269), .o1(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  nano22aa1n03x5               g185(.a(new_n273), .b(new_n279), .c(new_n280), .out0(new_n281));
  oaih12aa1n02x5               g186(.a(new_n271), .b(new_n276), .c(new_n275), .o1(new_n282));
  aoi012aa1n03x5               g187(.a(new_n280), .b(new_n282), .c(new_n279), .o1(new_n283));
  nor002aa1n02x5               g188(.a(new_n283), .b(new_n281), .o1(\s[28] ));
  nano22aa1n03x7               g189(.a(new_n280), .b(new_n279), .c(new_n270), .out0(new_n285));
  oai012aa1n06x5               g190(.a(new_n285), .b(new_n276), .c(new_n275), .o1(new_n286));
  oao003aa1n03x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n279), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  aoi012aa1n03x5               g193(.a(new_n288), .b(new_n286), .c(new_n287), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n285), .o1(new_n290));
  aoi013aa1n02x5               g195(.a(new_n290), .b(new_n265), .c(new_n266), .d(new_n268), .o1(new_n291));
  nano22aa1n03x5               g196(.a(new_n291), .b(new_n287), .c(new_n288), .out0(new_n292));
  norp02aa1n03x5               g197(.a(new_n289), .b(new_n292), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g199(.a(new_n288), .b(new_n280), .c(new_n270), .d(new_n279), .out0(new_n295));
  oai012aa1n06x5               g200(.a(new_n295), .b(new_n276), .c(new_n275), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .carry(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[29] ), .b(\a[30] ), .out0(new_n298));
  tech160nm_fiaoi012aa1n02p5x5 g203(.a(new_n298), .b(new_n296), .c(new_n297), .o1(new_n299));
  inv000aa1n02x5               g204(.a(new_n295), .o1(new_n300));
  aoi013aa1n02x5               g205(.a(new_n300), .b(new_n265), .c(new_n266), .d(new_n268), .o1(new_n301));
  nano22aa1n03x5               g206(.a(new_n301), .b(new_n297), .c(new_n298), .out0(new_n302));
  norp02aa1n03x5               g207(.a(new_n299), .b(new_n302), .o1(\s[30] ));
  norb03aa1n02x5               g208(.a(new_n285), .b(new_n298), .c(new_n288), .out0(new_n304));
  oaih12aa1n02x5               g209(.a(new_n304), .b(new_n276), .c(new_n275), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n297), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  inv000aa1n02x5               g213(.a(new_n304), .o1(new_n309));
  aoi013aa1n02x5               g214(.a(new_n309), .b(new_n265), .c(new_n266), .d(new_n268), .o1(new_n310));
  nano22aa1n03x5               g215(.a(new_n310), .b(new_n306), .c(new_n307), .out0(new_n311));
  norp02aa1n03x5               g216(.a(new_n308), .b(new_n311), .o1(\s[31] ));
  inv000aa1d42x5               g217(.a(\a[3] ), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n100), .b(\b[2] ), .c(new_n313), .out0(\s[3] ));
  xorc02aa1n02x5               g219(.a(\a[3] ), .b(\b[2] ), .out0(new_n315));
  nanb02aa1n03x5               g220(.a(new_n100), .b(new_n315), .out0(new_n316));
  xorc02aa1n02x5               g221(.a(\a[4] ), .b(\b[3] ), .out0(new_n317));
  aoib12aa1n02x5               g222(.a(new_n317), .b(new_n313), .c(\b[2] ), .out0(new_n318));
  aoi022aa1n02x5               g223(.a(new_n103), .b(new_n317), .c(new_n316), .d(new_n318), .o1(\s[4] ));
  aoi022aa1n02x5               g224(.a(new_n316), .b(new_n102), .c(\b[3] ), .d(\a[4] ), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g226(.a(new_n114), .b(new_n320), .c(new_n108), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanp02aa1n03x5               g228(.a(new_n320), .b(new_n108), .o1(new_n324));
  nona23aa1n02x4               g229(.a(new_n324), .b(new_n109), .c(new_n114), .d(new_n110), .out0(new_n325));
  nanb02aa1n02x5               g230(.a(new_n104), .b(new_n115), .out0(new_n326));
  xnbna2aa1n03x5               g231(.a(new_n326), .b(new_n325), .c(new_n109), .out0(\s[7] ));
  aoi013aa1n02x4               g232(.a(new_n104), .b(new_n325), .c(new_n115), .d(new_n109), .o1(new_n328));
  xnrb03aa1n03x5               g233(.a(new_n328), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrc02aa1n02x5               g234(.a(new_n118), .b(new_n134), .out0(\s[9] ));
endmodule


