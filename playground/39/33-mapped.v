// Benchmark "adder" written by ABC on Thu Jul 18 08:14:31 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n313, new_n315, new_n316, new_n317, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor002aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nand02aa1n03x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oao003aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n102));
  nor022aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand02aa1n03x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n06x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano23aa1n02x4               g011(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n107));
  nanp02aa1n02x5               g012(.a(new_n107), .b(new_n102), .o1(new_n108));
  tech160nm_fioai012aa1n03p5x5 g013(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n109));
  nor002aa1n03x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nand02aa1n03x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nanb02aa1n03x5               g016(.a(new_n110), .b(new_n111), .out0(new_n112));
  norp02aa1n06x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanb02aa1n02x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  nor022aa1n06x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand02aa1n03x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nor022aa1n16x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nanp02aa1n03x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nano23aa1n06x5               g024(.a(new_n116), .b(new_n118), .c(new_n119), .d(new_n117), .out0(new_n120));
  nona22aa1n02x4               g025(.a(new_n120), .b(new_n115), .c(new_n112), .out0(new_n121));
  tech160nm_fiao0012aa1n02p5x5 g026(.a(new_n110), .b(new_n113), .c(new_n111), .o(new_n122));
  oai012aa1n02x5               g027(.a(new_n117), .b(new_n118), .c(new_n116), .o1(new_n123));
  aobi12aa1n06x5               g028(.a(new_n123), .b(new_n120), .c(new_n122), .out0(new_n124));
  aoai13aa1n04x5               g029(.a(new_n124), .b(new_n121), .c(new_n108), .d(new_n109), .o1(new_n125));
  tech160nm_fixorc02aa1n02p5x5 g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoi012aa1n02x5               g031(.a(new_n98), .b(new_n125), .c(new_n126), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  xorc02aa1n02x5               g033(.a(\a[10] ), .b(\b[9] ), .out0(new_n129));
  and002aa1n02x5               g034(.a(new_n126), .b(new_n129), .o(new_n130));
  aob012aa1n02x5               g035(.a(new_n98), .b(\b[9] ), .c(\a[10] ), .out0(new_n131));
  oaib12aa1n06x5               g036(.a(new_n131), .b(\b[9] ), .c(new_n97), .out0(new_n132));
  nor002aa1n16x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  tech160nm_finand02aa1n05x5   g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n132), .c(new_n125), .d(new_n130), .o1(new_n136));
  aoi112aa1n02x5               g041(.a(new_n135), .b(new_n132), .c(new_n125), .d(new_n130), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(\s[11] ));
  inv000aa1d42x5               g043(.a(new_n133), .o1(new_n139));
  nor002aa1n03x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n136), .c(new_n139), .out0(\s[12] ));
  oaoi03aa1n06x5               g048(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n144));
  nona23aa1n03x5               g049(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n145));
  oai012aa1n06x5               g050(.a(new_n109), .b(new_n145), .c(new_n144), .o1(new_n146));
  nona23aa1n02x4               g051(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n147));
  nor043aa1n03x5               g052(.a(new_n147), .b(new_n115), .c(new_n112), .o1(new_n148));
  nanp02aa1n03x5               g053(.a(new_n146), .b(new_n148), .o1(new_n149));
  nano23aa1n06x5               g054(.a(new_n133), .b(new_n140), .c(new_n141), .d(new_n134), .out0(new_n150));
  nand23aa1n02x5               g055(.a(new_n150), .b(new_n129), .c(new_n126), .o1(new_n151));
  oai012aa1n02x5               g056(.a(new_n141), .b(new_n140), .c(new_n133), .o1(new_n152));
  aobi12aa1n06x5               g057(.a(new_n152), .b(new_n150), .c(new_n132), .out0(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n151), .c(new_n149), .d(new_n124), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nanp02aa1n04x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n156), .b(new_n154), .c(new_n157), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n08x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand02aa1n16x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nano23aa1n06x5               g066(.a(new_n156), .b(new_n160), .c(new_n161), .d(new_n157), .out0(new_n162));
  aoi012aa1d24x5               g067(.a(new_n160), .b(new_n156), .c(new_n161), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  xorc02aa1n12x5               g069(.a(\a[15] ), .b(\b[14] ), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n164), .c(new_n154), .d(new_n162), .o1(new_n166));
  aoi112aa1n02x5               g071(.a(new_n165), .b(new_n164), .c(new_n154), .d(new_n162), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(\s[15] ));
  nor042aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  xorc02aa1n12x5               g075(.a(\a[16] ), .b(\b[15] ), .out0(new_n171));
  aobi12aa1n02x5               g076(.a(new_n171), .b(new_n166), .c(new_n170), .out0(new_n172));
  nona22aa1n02x4               g077(.a(new_n166), .b(new_n171), .c(new_n169), .out0(new_n173));
  norb02aa1n03x4               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  aob012aa1n03x5               g079(.a(new_n123), .b(new_n120), .c(new_n122), .out0(new_n175));
  nand23aa1n03x5               g080(.a(new_n162), .b(new_n165), .c(new_n171), .o1(new_n176));
  nor042aa1n03x5               g081(.a(new_n176), .b(new_n151), .o1(new_n177));
  aoai13aa1n12x5               g082(.a(new_n177), .b(new_n175), .c(new_n146), .d(new_n148), .o1(new_n178));
  nanb03aa1n02x5               g083(.a(new_n163), .b(new_n171), .c(new_n165), .out0(new_n179));
  oaoi03aa1n02x5               g084(.a(\a[16] ), .b(\b[15] ), .c(new_n170), .o1(new_n180));
  nanb02aa1n06x5               g085(.a(new_n180), .b(new_n179), .out0(new_n181));
  oab012aa1n09x5               g086(.a(new_n181), .b(new_n153), .c(new_n176), .out0(new_n182));
  nanp02aa1n09x5               g087(.a(new_n178), .b(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g089(.a(\a[18] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\b[16] ), .o1(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d06x4               g094(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n190));
  nor042aa1n04x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  aoi112aa1n09x5               g096(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n192));
  nor042aa1n06x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  inv000aa1d42x5               g098(.a(new_n193), .o1(new_n194));
  nor002aa1n10x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanp02aa1n06x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  norb02aa1n06x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  aoai13aa1n06x5               g102(.a(new_n197), .b(new_n194), .c(new_n183), .d(new_n190), .o1(new_n198));
  aoi112aa1n02x5               g103(.a(new_n197), .b(new_n194), .c(new_n183), .d(new_n190), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nand42aa1d28x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n15x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  oaoi13aa1n06x5               g110(.a(new_n205), .b(new_n198), .c(\a[19] ), .d(\b[18] ), .o1(new_n206));
  nona22aa1n02x5               g111(.a(new_n198), .b(new_n204), .c(new_n195), .out0(new_n207));
  norb02aa1n03x4               g112(.a(new_n207), .b(new_n206), .out0(\s[20] ));
  nona23aa1n09x5               g113(.a(new_n203), .b(new_n196), .c(new_n195), .d(new_n202), .out0(new_n209));
  inv040aa1n02x5               g114(.a(new_n209), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(new_n190), .b(new_n210), .o1(new_n211));
  oai012aa1n09x5               g116(.a(new_n203), .b(new_n202), .c(new_n195), .o1(new_n212));
  oai012aa1n18x5               g117(.a(new_n212), .b(new_n209), .c(new_n193), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n04x5               g119(.a(new_n214), .b(new_n211), .c(new_n178), .d(new_n182), .o1(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n12x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n12x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xorc02aa1n12x5               g123(.a(\a[22] ), .b(\b[21] ), .out0(new_n219));
  aoai13aa1n03x5               g124(.a(new_n219), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n220));
  aoi112aa1n02x5               g125(.a(new_n217), .b(new_n219), .c(new_n215), .d(new_n218), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n220), .b(new_n221), .out0(\s[22] ));
  nand22aa1n12x5               g127(.a(new_n219), .b(new_n218), .o1(new_n223));
  nanb03aa1n06x5               g128(.a(new_n223), .b(new_n190), .c(new_n210), .out0(new_n224));
  oai112aa1n06x5               g129(.a(new_n197), .b(new_n204), .c(new_n192), .d(new_n191), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\a[22] ), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\b[21] ), .o1(new_n227));
  oaoi03aa1n12x5               g132(.a(new_n226), .b(new_n227), .c(new_n217), .o1(new_n228));
  aoai13aa1n12x5               g133(.a(new_n228), .b(new_n223), .c(new_n225), .d(new_n212), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoai13aa1n04x5               g135(.a(new_n230), .b(new_n224), .c(new_n178), .d(new_n182), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n236));
  aoi112aa1n02x5               g141(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n236), .b(new_n237), .out0(\s[24] ));
  and002aa1n06x5               g143(.a(new_n235), .b(new_n234), .o(new_n239));
  nona23aa1n02x4               g144(.a(new_n239), .b(new_n190), .c(new_n223), .d(new_n209), .out0(new_n240));
  inv000aa1d42x5               g145(.a(\a[24] ), .o1(new_n241));
  inv000aa1d42x5               g146(.a(\b[23] ), .o1(new_n242));
  oao003aa1n02x5               g147(.a(new_n241), .b(new_n242), .c(new_n233), .carry(new_n243));
  tech160nm_fiaoi012aa1n05x5   g148(.a(new_n243), .b(new_n229), .c(new_n239), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n240), .c(new_n178), .d(new_n182), .o1(new_n245));
  xorb03aa1n02x5               g150(.a(new_n245), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g151(.a(\b[24] ), .b(\a[25] ), .o1(new_n247));
  xorc02aa1n12x5               g152(.a(\a[25] ), .b(\b[24] ), .out0(new_n248));
  tech160nm_fixorc02aa1n04x5   g153(.a(\a[26] ), .b(\b[25] ), .out0(new_n249));
  aoai13aa1n03x5               g154(.a(new_n249), .b(new_n247), .c(new_n245), .d(new_n248), .o1(new_n250));
  aoi112aa1n03x5               g155(.a(new_n247), .b(new_n249), .c(new_n245), .d(new_n248), .o1(new_n251));
  norb02aa1n03x4               g156(.a(new_n250), .b(new_n251), .out0(\s[26] ));
  aoi013aa1n02x4               g157(.a(new_n180), .b(new_n164), .c(new_n165), .d(new_n171), .o1(new_n253));
  oai012aa1n02x5               g158(.a(new_n253), .b(new_n153), .c(new_n176), .o1(new_n254));
  and002aa1n18x5               g159(.a(new_n249), .b(new_n248), .o(new_n255));
  nano22aa1n03x7               g160(.a(new_n224), .b(new_n239), .c(new_n255), .out0(new_n256));
  aoai13aa1n04x5               g161(.a(new_n256), .b(new_n254), .c(new_n125), .d(new_n177), .o1(new_n257));
  aoai13aa1n09x5               g162(.a(new_n255), .b(new_n243), .c(new_n229), .d(new_n239), .o1(new_n258));
  oai022aa1n02x5               g163(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n259));
  aob012aa1n02x5               g164(.a(new_n259), .b(\b[25] ), .c(\a[26] ), .out0(new_n260));
  xorc02aa1n12x5               g165(.a(\a[27] ), .b(\b[26] ), .out0(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  aoi013aa1n06x4               g167(.a(new_n262), .b(new_n257), .c(new_n258), .d(new_n260), .o1(new_n263));
  inv000aa1n02x5               g168(.a(new_n256), .o1(new_n264));
  aoi012aa1n09x5               g169(.a(new_n264), .b(new_n178), .c(new_n182), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n223), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n228), .o1(new_n267));
  aoai13aa1n06x5               g172(.a(new_n239), .b(new_n267), .c(new_n213), .d(new_n266), .o1(new_n268));
  inv000aa1n02x5               g173(.a(new_n243), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n255), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n260), .b(new_n270), .c(new_n268), .d(new_n269), .o1(new_n271));
  norp03aa1n02x5               g176(.a(new_n271), .b(new_n265), .c(new_n261), .o1(new_n272));
  norp02aa1n02x5               g177(.a(new_n263), .b(new_n272), .o1(\s[27] ));
  norp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  inv040aa1n03x5               g179(.a(new_n274), .o1(new_n275));
  oaih12aa1n02x5               g180(.a(new_n261), .b(new_n271), .c(new_n265), .o1(new_n276));
  xnrc02aa1n12x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  tech160nm_fiaoi012aa1n02p5x5 g182(.a(new_n277), .b(new_n276), .c(new_n275), .o1(new_n278));
  nano22aa1n03x7               g183(.a(new_n263), .b(new_n275), .c(new_n277), .out0(new_n279));
  norp02aa1n03x5               g184(.a(new_n278), .b(new_n279), .o1(\s[28] ));
  norb02aa1d21x5               g185(.a(new_n261), .b(new_n277), .out0(new_n281));
  oai012aa1n02x5               g186(.a(new_n281), .b(new_n271), .c(new_n265), .o1(new_n282));
  oao003aa1n02x5               g187(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  aoi012aa1n03x5               g189(.a(new_n284), .b(new_n282), .c(new_n283), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n281), .o1(new_n286));
  aoi013aa1n02x5               g191(.a(new_n286), .b(new_n257), .c(new_n258), .d(new_n260), .o1(new_n287));
  nano22aa1n03x5               g192(.a(new_n287), .b(new_n283), .c(new_n284), .out0(new_n288));
  norp02aa1n03x5               g193(.a(new_n285), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n09x5               g195(.a(new_n261), .b(new_n284), .c(new_n277), .out0(new_n291));
  oai012aa1n02x5               g196(.a(new_n291), .b(new_n271), .c(new_n265), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n283), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  tech160nm_fiaoi012aa1n02p5x5 g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n291), .o1(new_n296));
  aoi013aa1n02x5               g201(.a(new_n296), .b(new_n257), .c(new_n258), .d(new_n260), .o1(new_n297));
  nano22aa1n03x5               g202(.a(new_n297), .b(new_n293), .c(new_n294), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n295), .b(new_n298), .o1(\s[30] ));
  xnrc02aa1n02x5               g204(.a(\b[30] ), .b(\a[31] ), .out0(new_n300));
  norb02aa1n03x4               g205(.a(new_n291), .b(new_n294), .out0(new_n301));
  oaih12aa1n02x5               g206(.a(new_n301), .b(new_n271), .c(new_n265), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n300), .b(new_n302), .c(new_n303), .o1(new_n304));
  inv020aa1n02x5               g209(.a(new_n301), .o1(new_n305));
  aoi013aa1n02x5               g210(.a(new_n305), .b(new_n257), .c(new_n258), .d(new_n260), .o1(new_n306));
  nano22aa1n03x5               g211(.a(new_n306), .b(new_n300), .c(new_n303), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n304), .b(new_n307), .o1(\s[31] ));
  xnrb03aa1n02x5               g213(.a(new_n144), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g214(.a(\a[3] ), .b(\b[2] ), .c(new_n144), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n146), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fiao0012aa1n02p5x5 g217(.a(new_n113), .b(new_n146), .c(new_n114), .o(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g219(.a(new_n119), .b(new_n118), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n110), .c(new_n313), .d(new_n111), .o1(new_n316));
  aoi112aa1n02x5               g221(.a(new_n315), .b(new_n110), .c(new_n313), .d(new_n111), .o1(new_n317));
  norb02aa1n02x5               g222(.a(new_n316), .b(new_n317), .out0(\s[7] ));
  oai012aa1n02x5               g223(.a(new_n316), .b(\b[6] ), .c(\a[7] ), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g225(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


