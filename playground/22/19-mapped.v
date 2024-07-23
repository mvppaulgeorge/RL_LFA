// Benchmark "adder" written by ABC on Wed Jul 17 23:23:18 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n307, new_n308, new_n310,
    new_n313, new_n315, new_n317;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n04x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  tech160nm_finor002aa1n05x5   g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\a[7] ), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\b[6] ), .o1(new_n102));
  nanp02aa1n04x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  nor002aa1d32x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  aoai13aa1n02x5               g009(.a(new_n103), .b(new_n104), .c(new_n102), .d(new_n101), .o1(new_n105));
  inv000aa1d42x5               g010(.a(\a[5] ), .o1(new_n106));
  inv000aa1d42x5               g011(.a(\a[6] ), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\b[4] ), .o1(new_n108));
  aboi22aa1n02x7               g013(.a(\b[5] ), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n109));
  and002aa1n02x5               g014(.a(\b[5] ), .b(\a[6] ), .o(new_n110));
  nor042aa1d18x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n04x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nona23aa1n09x5               g017(.a(new_n103), .b(new_n112), .c(new_n111), .d(new_n104), .out0(new_n113));
  oai013aa1n03x5               g018(.a(new_n105), .b(new_n113), .c(new_n109), .d(new_n110), .o1(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n02x5               g021(.a(new_n113), .b(new_n115), .c(new_n116), .o1(new_n117));
  and002aa1n02x5               g022(.a(\b[3] ), .b(\a[4] ), .o(new_n118));
  nor002aa1d32x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  oab012aa1n02x5               g024(.a(new_n119), .b(\a[4] ), .c(\b[3] ), .out0(new_n120));
  nand02aa1d20x5               g025(.a(\b[1] ), .b(\a[2] ), .o1(new_n121));
  nand42aa1n16x5               g026(.a(\b[2] ), .b(\a[3] ), .o1(new_n122));
  nanb03aa1d24x5               g027(.a(new_n119), .b(new_n122), .c(new_n121), .out0(new_n123));
  nand02aa1d06x5               g028(.a(\b[0] ), .b(\a[1] ), .o1(new_n124));
  nor042aa1n04x5               g029(.a(\b[1] ), .b(\a[2] ), .o1(new_n125));
  norb03aa1d15x5               g030(.a(new_n121), .b(new_n125), .c(new_n124), .out0(new_n126));
  oaoi13aa1n12x5               g031(.a(new_n118), .b(new_n120), .c(new_n126), .d(new_n123), .o1(new_n127));
  tech160nm_fiao0012aa1n02p5x5 g032(.a(new_n114), .b(new_n127), .c(new_n117), .o(new_n128));
  nand42aa1n10x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  aoai13aa1n02x5               g034(.a(new_n99), .b(new_n100), .c(new_n128), .d(new_n129), .o1(new_n130));
  inv000aa1n02x5               g035(.a(new_n98), .o1(new_n131));
  norb02aa1n06x4               g036(.a(new_n129), .b(new_n100), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n114), .c(new_n127), .d(new_n117), .o1(new_n133));
  nona32aa1n02x4               g038(.a(new_n133), .b(new_n100), .c(new_n131), .d(new_n97), .out0(new_n134));
  nanp02aa1n02x5               g039(.a(new_n130), .b(new_n134), .o1(\s[10] ));
  xnrc02aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n134), .c(new_n98), .out0(\s[11] ));
  inv000aa1d42x5               g042(.a(\a[11] ), .o1(new_n138));
  inv000aa1d42x5               g043(.a(\b[10] ), .o1(new_n139));
  nand02aa1d08x5               g044(.a(new_n139), .b(new_n138), .o1(new_n140));
  nona22aa1n02x4               g045(.a(new_n134), .b(new_n136), .c(new_n131), .out0(new_n141));
  norp02aa1n04x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  and002aa1n03x5               g047(.a(\b[11] ), .b(\a[12] ), .o(new_n143));
  norp02aa1n02x5               g048(.a(new_n143), .b(new_n142), .o1(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n141), .c(new_n140), .out0(\s[12] ));
  nona23aa1n09x5               g050(.a(new_n132), .b(new_n140), .c(new_n131), .d(new_n97), .out0(new_n146));
  aoi112aa1n03x5               g051(.a(new_n143), .b(new_n142), .c(\a[11] ), .d(\b[10] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  aoai13aa1n03x5               g053(.a(new_n148), .b(new_n114), .c(new_n127), .d(new_n117), .o1(new_n149));
  oai022aa1n02x5               g054(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n150));
  aoi022aa1d24x5               g055(.a(new_n138), .b(new_n139), .c(\a[10] ), .d(\b[9] ), .o1(new_n151));
  oaoi03aa1n06x5               g056(.a(\a[12] ), .b(\b[11] ), .c(new_n140), .o1(new_n152));
  aoi013aa1n02x4               g057(.a(new_n152), .b(new_n147), .c(new_n150), .d(new_n151), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n149), .b(new_n153), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n20x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand42aa1d28x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n156), .b(new_n154), .c(new_n157), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n06x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand42aa1n20x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nano23aa1d15x5               g066(.a(new_n156), .b(new_n160), .c(new_n161), .d(new_n157), .out0(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  oai012aa1n02x5               g068(.a(new_n161), .b(new_n160), .c(new_n156), .o1(new_n164));
  aoai13aa1n04x5               g069(.a(new_n164), .b(new_n163), .c(new_n149), .d(new_n153), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  xorc02aa1n12x5               g072(.a(\a[15] ), .b(\b[14] ), .out0(new_n168));
  xorc02aa1n12x5               g073(.a(\a[16] ), .b(\b[15] ), .out0(new_n169));
  aoai13aa1n03x5               g074(.a(new_n169), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n170));
  aoi112aa1n02x5               g075(.a(new_n167), .b(new_n169), .c(new_n165), .d(new_n168), .o1(new_n171));
  norb02aa1n02x7               g076(.a(new_n170), .b(new_n171), .out0(\s[16] ));
  nanp02aa1n02x5               g077(.a(new_n169), .b(new_n168), .o1(new_n173));
  nano23aa1n06x5               g078(.a(new_n173), .b(new_n146), .c(new_n162), .d(new_n147), .out0(new_n174));
  aoai13aa1n12x5               g079(.a(new_n174), .b(new_n114), .c(new_n127), .d(new_n117), .o1(new_n175));
  nanb03aa1n02x5               g080(.a(new_n164), .b(new_n169), .c(new_n168), .out0(new_n176));
  nanp03aa1n02x5               g081(.a(new_n147), .b(new_n150), .c(new_n151), .o1(new_n177));
  inv000aa1n02x5               g082(.a(new_n152), .o1(new_n178));
  nand23aa1n03x5               g083(.a(new_n162), .b(new_n168), .c(new_n169), .o1(new_n179));
  tech160nm_fiaoi012aa1n04x5   g084(.a(new_n179), .b(new_n177), .c(new_n178), .o1(new_n180));
  oai022aa1n02x5               g085(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n181));
  aob012aa1n02x5               g086(.a(new_n181), .b(\b[15] ), .c(\a[16] ), .out0(new_n182));
  nano22aa1d15x5               g087(.a(new_n180), .b(new_n176), .c(new_n182), .out0(new_n183));
  nanp02aa1n12x5               g088(.a(new_n175), .b(new_n183), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g090(.a(\a[18] ), .o1(new_n186));
  inv040aa1d32x5               g091(.a(\a[17] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\b[16] ), .o1(new_n188));
  oaoi03aa1n03x5               g093(.a(new_n187), .b(new_n188), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n186), .out0(\s[18] ));
  xroi22aa1d06x4               g095(.a(new_n187), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n191));
  nanp02aa1n02x5               g096(.a(new_n188), .b(new_n187), .o1(new_n192));
  oaoi03aa1n12x5               g097(.a(\a[18] ), .b(\b[17] ), .c(new_n192), .o1(new_n193));
  nor022aa1n08x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nanp02aa1n12x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n193), .c(new_n184), .d(new_n191), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(new_n196), .b(new_n193), .c(new_n184), .d(new_n191), .o1(new_n198));
  norb02aa1n02x7               g103(.a(new_n197), .b(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  orn002aa1n24x5               g105(.a(\a[19] ), .b(\b[18] ), .o(new_n201));
  norp02aa1n12x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nand42aa1n06x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  aobi12aa1n06x5               g109(.a(new_n204), .b(new_n197), .c(new_n201), .out0(new_n205));
  nona22aa1n02x5               g110(.a(new_n197), .b(new_n204), .c(new_n194), .out0(new_n206));
  norb02aa1n03x4               g111(.a(new_n206), .b(new_n205), .out0(\s[20] ));
  nano23aa1n03x7               g112(.a(new_n194), .b(new_n202), .c(new_n203), .d(new_n195), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n191), .b(new_n208), .o1(new_n209));
  oai022aa1n02x7               g114(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n210));
  oaib12aa1n12x5               g115(.a(new_n210), .b(new_n186), .c(\b[17] ), .out0(new_n211));
  nona23aa1n06x5               g116(.a(new_n203), .b(new_n195), .c(new_n194), .d(new_n202), .out0(new_n212));
  oaoi03aa1n12x5               g117(.a(\a[20] ), .b(\b[19] ), .c(new_n201), .o1(new_n213));
  inv040aa1n02x5               g118(.a(new_n213), .o1(new_n214));
  oai012aa1n18x5               g119(.a(new_n214), .b(new_n212), .c(new_n211), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  aoai13aa1n04x5               g121(.a(new_n216), .b(new_n209), .c(new_n175), .d(new_n183), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  xorc02aa1n02x5               g124(.a(\a[21] ), .b(\b[20] ), .out0(new_n220));
  xorc02aa1n02x5               g125(.a(\a[22] ), .b(\b[21] ), .out0(new_n221));
  aoai13aa1n03x5               g126(.a(new_n221), .b(new_n219), .c(new_n217), .d(new_n220), .o1(new_n222));
  aoi112aa1n02x5               g127(.a(new_n219), .b(new_n221), .c(new_n217), .d(new_n220), .o1(new_n223));
  norb02aa1n02x7               g128(.a(new_n222), .b(new_n223), .out0(\s[22] ));
  inv000aa1d42x5               g129(.a(\a[21] ), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\a[22] ), .o1(new_n226));
  xroi22aa1d06x4               g131(.a(new_n225), .b(\b[20] ), .c(new_n226), .d(\b[21] ), .out0(new_n227));
  nand03aa1n02x5               g132(.a(new_n227), .b(new_n191), .c(new_n208), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\b[21] ), .o1(new_n229));
  oao003aa1n02x5               g134(.a(new_n226), .b(new_n229), .c(new_n219), .carry(new_n230));
  aoi012aa1n02x5               g135(.a(new_n230), .b(new_n215), .c(new_n227), .o1(new_n231));
  aoai13aa1n04x5               g136(.a(new_n231), .b(new_n228), .c(new_n175), .d(new_n183), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n02x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  xorc02aa1n02x5               g140(.a(\a[24] ), .b(\b[23] ), .out0(new_n236));
  aoai13aa1n03x5               g141(.a(new_n236), .b(new_n234), .c(new_n232), .d(new_n235), .o1(new_n237));
  aoi112aa1n02x5               g142(.a(new_n234), .b(new_n236), .c(new_n232), .d(new_n235), .o1(new_n238));
  norb02aa1n02x7               g143(.a(new_n237), .b(new_n238), .out0(\s[24] ));
  aoai13aa1n04x5               g144(.a(new_n227), .b(new_n213), .c(new_n208), .d(new_n193), .o1(new_n240));
  inv000aa1n02x5               g145(.a(new_n230), .o1(new_n241));
  and002aa1n02x5               g146(.a(new_n236), .b(new_n235), .o(new_n242));
  inv000aa1n02x5               g147(.a(new_n242), .o1(new_n243));
  oai022aa1n02x5               g148(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n244));
  aob012aa1n02x5               g149(.a(new_n244), .b(\b[23] ), .c(\a[24] ), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n243), .c(new_n240), .d(new_n241), .o1(new_n246));
  nano32aa1n02x4               g151(.a(new_n243), .b(new_n227), .c(new_n191), .d(new_n208), .out0(new_n247));
  xorc02aa1n12x5               g152(.a(\a[25] ), .b(\b[24] ), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n246), .c(new_n184), .d(new_n247), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n246), .b(new_n248), .c(new_n184), .d(new_n247), .o1(new_n250));
  norb02aa1n02x7               g155(.a(new_n249), .b(new_n250), .out0(\s[25] ));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  xorc02aa1n12x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  aobi12aa1n06x5               g159(.a(new_n254), .b(new_n249), .c(new_n253), .out0(new_n255));
  nona22aa1n02x5               g160(.a(new_n249), .b(new_n254), .c(new_n252), .out0(new_n256));
  norb02aa1n03x4               g161(.a(new_n256), .b(new_n255), .out0(\s[26] ));
  and002aa1n06x5               g162(.a(new_n254), .b(new_n248), .o(new_n258));
  nano22aa1n03x7               g163(.a(new_n228), .b(new_n242), .c(new_n258), .out0(new_n259));
  nand22aa1n06x5               g164(.a(new_n184), .b(new_n259), .o1(new_n260));
  oao003aa1n02x5               g165(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n261));
  aobi12aa1n09x5               g166(.a(new_n261), .b(new_n246), .c(new_n258), .out0(new_n262));
  xorc02aa1n12x5               g167(.a(\a[27] ), .b(\b[26] ), .out0(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n260), .c(new_n262), .out0(\s[27] ));
  nor042aa1n03x5               g169(.a(\b[26] ), .b(\a[27] ), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  inv040aa1n02x5               g171(.a(new_n259), .o1(new_n267));
  tech160nm_fiaoi012aa1n05x5   g172(.a(new_n267), .b(new_n175), .c(new_n183), .o1(new_n268));
  aoai13aa1n04x5               g173(.a(new_n242), .b(new_n230), .c(new_n215), .d(new_n227), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n258), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n261), .b(new_n270), .c(new_n269), .d(new_n245), .o1(new_n271));
  oaih12aa1n02x5               g176(.a(new_n263), .b(new_n271), .c(new_n268), .o1(new_n272));
  xnrc02aa1n12x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  tech160nm_fiaoi012aa1n03p5x5 g178(.a(new_n273), .b(new_n272), .c(new_n266), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n263), .o1(new_n275));
  tech160nm_fiaoi012aa1n03p5x5 g180(.a(new_n275), .b(new_n260), .c(new_n262), .o1(new_n276));
  nano22aa1n03x5               g181(.a(new_n276), .b(new_n266), .c(new_n273), .out0(new_n277));
  norp02aa1n03x5               g182(.a(new_n274), .b(new_n277), .o1(\s[28] ));
  norb02aa1d21x5               g183(.a(new_n263), .b(new_n273), .out0(new_n279));
  oaih12aa1n02x5               g184(.a(new_n279), .b(new_n271), .c(new_n268), .o1(new_n280));
  oao003aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .c(new_n266), .carry(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  tech160nm_fiaoi012aa1n02p5x5 g187(.a(new_n282), .b(new_n280), .c(new_n281), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n279), .o1(new_n284));
  tech160nm_fiaoi012aa1n03p5x5 g189(.a(new_n284), .b(new_n260), .c(new_n262), .o1(new_n285));
  nano22aa1n03x5               g190(.a(new_n285), .b(new_n281), .c(new_n282), .out0(new_n286));
  norp02aa1n03x5               g191(.a(new_n283), .b(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g192(.a(new_n124), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g193(.a(new_n263), .b(new_n282), .c(new_n273), .out0(new_n289));
  oaih12aa1n02x5               g194(.a(new_n289), .b(new_n271), .c(new_n268), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n281), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[29] ), .b(\a[30] ), .out0(new_n292));
  tech160nm_fiaoi012aa1n02p5x5 g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n289), .o1(new_n294));
  tech160nm_fiaoi012aa1n03p5x5 g199(.a(new_n294), .b(new_n260), .c(new_n262), .o1(new_n295));
  nano22aa1n03x5               g200(.a(new_n295), .b(new_n291), .c(new_n292), .out0(new_n296));
  norp02aa1n03x5               g201(.a(new_n293), .b(new_n296), .o1(\s[30] ));
  norb02aa1n02x5               g202(.a(new_n289), .b(new_n292), .out0(new_n298));
  inv000aa1n02x5               g203(.a(new_n298), .o1(new_n299));
  tech160nm_fiaoi012aa1n03p5x5 g204(.a(new_n299), .b(new_n260), .c(new_n262), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .c(new_n291), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nano22aa1n03x5               g207(.a(new_n300), .b(new_n301), .c(new_n302), .out0(new_n303));
  oaih12aa1n02x5               g208(.a(new_n298), .b(new_n271), .c(new_n268), .o1(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n302), .b(new_n304), .c(new_n301), .o1(new_n305));
  norp02aa1n03x5               g210(.a(new_n305), .b(new_n303), .o1(\s[31] ));
  norb02aa1n02x5               g211(.a(new_n122), .b(new_n119), .out0(new_n307));
  oaoi13aa1n02x5               g212(.a(new_n307), .b(new_n121), .c(new_n125), .d(new_n124), .o1(new_n308));
  oab012aa1n02x4               g213(.a(new_n308), .b(new_n123), .c(new_n126), .out0(\s[3] ));
  oabi12aa1n02x5               g214(.a(new_n119), .b(new_n126), .c(new_n123), .out0(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n127), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g217(.a(new_n106), .b(new_n108), .c(new_n127), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(new_n107), .out0(\s[6] ));
  oaoi03aa1n02x5               g219(.a(\a[6] ), .b(\b[5] ), .c(new_n313), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g221(.a(new_n101), .b(new_n102), .c(new_n315), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g223(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


