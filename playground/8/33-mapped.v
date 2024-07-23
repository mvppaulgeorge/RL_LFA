// Benchmark "adder" written by ABC on Wed Jul 17 16:20:04 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n330, new_n332, new_n333, new_n334,
    new_n336, new_n337;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n03x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1n03x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  aoi012aa1n06x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  inv040aa1n03x5               g007(.a(new_n102), .o1(new_n103));
  nor042aa1n06x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand02aa1n03x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1n04x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand22aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nano23aa1n06x5               g012(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n105), .out0(new_n108));
  inv030aa1n02x5               g013(.a(new_n104), .o1(new_n109));
  aob012aa1n06x5               g014(.a(new_n109), .b(new_n106), .c(new_n105), .out0(new_n110));
  aoi012aa1n09x5               g015(.a(new_n110), .b(new_n108), .c(new_n103), .o1(new_n111));
  tech160nm_fixorc02aa1n04x5   g016(.a(\a[5] ), .b(\b[4] ), .out0(new_n112));
  nor042aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nand02aa1n06x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  norb02aa1n06x4               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  nor042aa1n02x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand42aa1n08x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  norp02aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nand42aa1n03x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nano23aa1n02x4               g024(.a(new_n116), .b(new_n118), .c(new_n119), .d(new_n117), .out0(new_n120));
  nand23aa1n03x5               g025(.a(new_n120), .b(new_n112), .c(new_n115), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\a[8] ), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\b[7] ), .o1(new_n123));
  inv020aa1n02x5               g028(.a(new_n116), .o1(new_n124));
  nor042aa1n02x5               g029(.a(\b[4] ), .b(\a[5] ), .o1(new_n125));
  aoai13aa1n03x5               g030(.a(new_n117), .b(new_n113), .c(new_n125), .d(new_n114), .o1(new_n126));
  nanp02aa1n03x5               g031(.a(new_n126), .b(new_n124), .o1(new_n127));
  oaoi03aa1n09x5               g032(.a(new_n122), .b(new_n123), .c(new_n127), .o1(new_n128));
  oai012aa1n18x5               g033(.a(new_n128), .b(new_n111), .c(new_n121), .o1(new_n129));
  xnrc02aa1n12x5               g034(.a(\b[8] ), .b(\a[9] ), .out0(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  aoi012aa1n02x5               g036(.a(new_n98), .b(new_n129), .c(new_n131), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  xnrc02aa1n02x5               g038(.a(\b[9] ), .b(\a[10] ), .out0(new_n134));
  norp02aa1n02x5               g039(.a(new_n130), .b(new_n134), .o1(new_n135));
  inv000aa1d42x5               g040(.a(\b[9] ), .o1(new_n136));
  oao003aa1n02x5               g041(.a(new_n97), .b(new_n136), .c(new_n98), .carry(new_n137));
  nor002aa1n16x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nanp02aa1n04x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  aoai13aa1n02x5               g045(.a(new_n140), .b(new_n137), .c(new_n129), .d(new_n135), .o1(new_n141));
  aoi112aa1n02x5               g046(.a(new_n140), .b(new_n137), .c(new_n129), .d(new_n135), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n141), .b(new_n142), .out0(\s[11] ));
  inv000aa1d42x5               g048(.a(new_n138), .o1(new_n144));
  nor042aa1n04x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nand42aa1n06x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nanb02aa1n02x5               g051(.a(new_n145), .b(new_n146), .out0(new_n147));
  xobna2aa1n03x5               g052(.a(new_n147), .b(new_n141), .c(new_n144), .out0(\s[12] ));
  nanb02aa1n02x5               g053(.a(new_n106), .b(new_n107), .out0(new_n149));
  nano23aa1n06x5               g054(.a(new_n102), .b(new_n149), .c(new_n109), .d(new_n105), .out0(new_n150));
  nanb02aa1n02x5               g055(.a(new_n116), .b(new_n117), .out0(new_n151));
  norb02aa1n02x5               g056(.a(new_n119), .b(new_n118), .out0(new_n152));
  nano32aa1n03x7               g057(.a(new_n151), .b(new_n112), .c(new_n152), .d(new_n115), .out0(new_n153));
  tech160nm_fioai012aa1n05x5   g058(.a(new_n153), .b(new_n150), .c(new_n110), .o1(new_n154));
  nano23aa1d15x5               g059(.a(new_n138), .b(new_n145), .c(new_n146), .d(new_n139), .out0(new_n155));
  nona22aa1d24x5               g060(.a(new_n155), .b(new_n130), .c(new_n134), .out0(new_n156));
  tech160nm_fioai012aa1n04x5   g061(.a(new_n146), .b(new_n145), .c(new_n138), .o1(new_n157));
  aobi12aa1n06x5               g062(.a(new_n157), .b(new_n155), .c(new_n137), .out0(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n156), .c(new_n154), .d(new_n128), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  tech160nm_finand02aa1n05x5   g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n161), .b(new_n159), .c(new_n162), .o1(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv000aa1d42x5               g069(.a(new_n156), .o1(new_n165));
  oaoi03aa1n02x5               g070(.a(new_n97), .b(new_n136), .c(new_n98), .o1(new_n166));
  nona23aa1n02x4               g071(.a(new_n146), .b(new_n139), .c(new_n138), .d(new_n145), .out0(new_n167));
  tech160nm_fioai012aa1n04x5   g072(.a(new_n157), .b(new_n167), .c(new_n166), .o1(new_n168));
  nor042aa1n06x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nanp02aa1n09x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nano23aa1n03x7               g075(.a(new_n161), .b(new_n169), .c(new_n170), .d(new_n162), .out0(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n168), .c(new_n129), .d(new_n165), .o1(new_n172));
  oai012aa1n12x5               g077(.a(new_n170), .b(new_n169), .c(new_n161), .o1(new_n173));
  tech160nm_fixorc02aa1n04x5   g078(.a(\a[15] ), .b(\b[14] ), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n172), .c(new_n173), .out0(\s[15] ));
  xnrc02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .out0(new_n176));
  nor042aa1n06x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  xnrc02aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .out0(new_n179));
  aoai13aa1n03x5               g084(.a(new_n178), .b(new_n179), .c(new_n172), .d(new_n173), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n180), .b(new_n176), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n173), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n174), .b(new_n182), .c(new_n159), .d(new_n171), .o1(new_n183));
  nona22aa1n02x4               g088(.a(new_n183), .b(new_n176), .c(new_n177), .out0(new_n184));
  nanp02aa1n02x5               g089(.a(new_n181), .b(new_n184), .o1(\s[16] ));
  xorc02aa1n02x5               g090(.a(\a[16] ), .b(\b[15] ), .out0(new_n186));
  nand23aa1n04x5               g091(.a(new_n171), .b(new_n174), .c(new_n186), .o1(new_n187));
  nor042aa1n06x5               g092(.a(new_n156), .b(new_n187), .o1(new_n188));
  nand22aa1n03x5               g093(.a(new_n129), .b(new_n188), .o1(new_n189));
  oao003aa1n02x5               g094(.a(\a[16] ), .b(\b[15] ), .c(new_n178), .carry(new_n190));
  oai013aa1n03x4               g095(.a(new_n190), .b(new_n179), .c(new_n176), .d(new_n173), .o1(new_n191));
  aoib12aa1n06x5               g096(.a(new_n191), .b(new_n168), .c(new_n187), .out0(new_n192));
  tech160nm_fixorc02aa1n03p5x5 g097(.a(\a[17] ), .b(\b[16] ), .out0(new_n193));
  xnbna2aa1n03x5               g098(.a(new_n193), .b(new_n189), .c(new_n192), .out0(\s[17] ));
  inv000aa1d42x5               g099(.a(\a[17] ), .o1(new_n195));
  nanb02aa1n02x5               g100(.a(\b[16] ), .b(new_n195), .out0(new_n196));
  oabi12aa1n06x5               g101(.a(new_n191), .b(new_n158), .c(new_n187), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n193), .b(new_n197), .c(new_n129), .d(new_n188), .o1(new_n198));
  tech160nm_fixorc02aa1n05x5   g103(.a(\a[18] ), .b(\b[17] ), .out0(new_n199));
  xnbna2aa1n03x5               g104(.a(new_n199), .b(new_n198), .c(new_n196), .out0(\s[18] ));
  inv040aa1d32x5               g105(.a(\a[18] ), .o1(new_n201));
  xroi22aa1d04x5               g106(.a(new_n195), .b(\b[16] ), .c(new_n201), .d(\b[17] ), .out0(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n197), .c(new_n129), .d(new_n188), .o1(new_n203));
  oai022aa1n02x7               g108(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n204));
  oaib12aa1n09x5               g109(.a(new_n204), .b(new_n201), .c(\b[17] ), .out0(new_n205));
  nor022aa1n12x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nanp02aa1n04x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nanb02aa1n12x5               g112(.a(new_n206), .b(new_n207), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n203), .c(new_n205), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g116(.a(new_n203), .b(new_n205), .o1(new_n212));
  nor042aa1n02x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand02aa1n06x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nanb02aa1n12x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  aoai13aa1n03x5               g120(.a(new_n215), .b(new_n206), .c(new_n212), .d(new_n209), .o1(new_n216));
  inv020aa1n04x5               g121(.a(new_n188), .o1(new_n217));
  aoai13aa1n12x5               g122(.a(new_n192), .b(new_n217), .c(new_n154), .d(new_n128), .o1(new_n218));
  oaoi03aa1n02x5               g123(.a(\a[18] ), .b(\b[17] ), .c(new_n196), .o1(new_n219));
  aoai13aa1n03x5               g124(.a(new_n209), .b(new_n219), .c(new_n218), .d(new_n202), .o1(new_n220));
  nona22aa1n02x4               g125(.a(new_n220), .b(new_n215), .c(new_n206), .out0(new_n221));
  nanp02aa1n02x5               g126(.a(new_n216), .b(new_n221), .o1(\s[20] ));
  nano23aa1n06x5               g127(.a(new_n206), .b(new_n213), .c(new_n214), .d(new_n207), .out0(new_n223));
  nand23aa1n06x5               g128(.a(new_n223), .b(new_n193), .c(new_n199), .o1(new_n224));
  oai022aa1n02x5               g129(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n225));
  aoi022aa1n06x5               g130(.a(new_n223), .b(new_n219), .c(new_n214), .d(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n224), .c(new_n189), .d(new_n192), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  xnrc02aa1n12x5               g134(.a(\b[20] ), .b(\a[21] ), .out0(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  tech160nm_fixnrc02aa1n05x5   g136(.a(\b[21] ), .b(\a[22] ), .out0(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n229), .c(new_n227), .d(new_n231), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n224), .o1(new_n234));
  oai012aa1n02x5               g139(.a(new_n214), .b(new_n213), .c(new_n206), .o1(new_n235));
  oai013aa1d12x5               g140(.a(new_n235), .b(new_n205), .c(new_n208), .d(new_n215), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n231), .b(new_n236), .c(new_n218), .d(new_n234), .o1(new_n237));
  nona22aa1n02x4               g142(.a(new_n237), .b(new_n232), .c(new_n229), .out0(new_n238));
  nanp02aa1n02x5               g143(.a(new_n233), .b(new_n238), .o1(\s[22] ));
  nor042aa1n06x5               g144(.a(new_n232), .b(new_n230), .o1(new_n240));
  norb02aa1n06x4               g145(.a(new_n240), .b(new_n224), .out0(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n197), .c(new_n129), .d(new_n188), .o1(new_n242));
  inv000aa1d42x5               g147(.a(\a[22] ), .o1(new_n243));
  inv040aa1d32x5               g148(.a(\b[21] ), .o1(new_n244));
  oaoi03aa1n12x5               g149(.a(new_n243), .b(new_n244), .c(new_n229), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  aoi012aa1n02x5               g151(.a(new_n246), .b(new_n236), .c(new_n240), .o1(new_n247));
  nor042aa1n03x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  nanp02aa1n03x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  nanb02aa1n02x5               g154(.a(new_n248), .b(new_n249), .out0(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  xnbna2aa1n03x5               g156(.a(new_n251), .b(new_n242), .c(new_n247), .out0(\s[23] ));
  nanp02aa1n02x5               g157(.a(new_n242), .b(new_n247), .o1(new_n253));
  nor042aa1n02x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  nanp02aa1n04x5               g159(.a(\b[23] ), .b(\a[24] ), .o1(new_n255));
  nanb02aa1n02x5               g160(.a(new_n254), .b(new_n255), .out0(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n248), .c(new_n253), .d(new_n251), .o1(new_n257));
  inv000aa1n02x5               g162(.a(new_n247), .o1(new_n258));
  aoai13aa1n03x5               g163(.a(new_n251), .b(new_n258), .c(new_n218), .d(new_n241), .o1(new_n259));
  nona22aa1n03x5               g164(.a(new_n259), .b(new_n256), .c(new_n248), .out0(new_n260));
  nanp02aa1n02x5               g165(.a(new_n257), .b(new_n260), .o1(\s[24] ));
  nona23aa1d18x5               g166(.a(new_n255), .b(new_n249), .c(new_n248), .d(new_n254), .out0(new_n262));
  inv040aa1n03x5               g167(.a(new_n262), .o1(new_n263));
  nano22aa1n03x7               g168(.a(new_n224), .b(new_n263), .c(new_n240), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n197), .c(new_n129), .d(new_n188), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(new_n248), .b(new_n255), .o1(new_n266));
  oai122aa1n06x5               g171(.a(new_n266), .b(new_n262), .c(new_n245), .d(\b[23] ), .e(\a[24] ), .o1(new_n267));
  aoi013aa1n09x5               g172(.a(new_n267), .b(new_n236), .c(new_n240), .d(new_n263), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n265), .c(new_n268), .out0(\s[25] ));
  nanp02aa1n02x5               g175(.a(new_n265), .b(new_n268), .o1(new_n271));
  norp02aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  xnrc02aa1n06x5               g177(.a(\b[25] ), .b(\a[26] ), .out0(new_n273));
  aoai13aa1n02x5               g178(.a(new_n273), .b(new_n272), .c(new_n271), .d(new_n269), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n268), .o1(new_n275));
  aoai13aa1n03x5               g180(.a(new_n269), .b(new_n275), .c(new_n218), .d(new_n264), .o1(new_n276));
  nona22aa1n03x5               g181(.a(new_n276), .b(new_n273), .c(new_n272), .out0(new_n277));
  nanp02aa1n02x5               g182(.a(new_n274), .b(new_n277), .o1(\s[26] ));
  norb02aa1n06x5               g183(.a(new_n269), .b(new_n273), .out0(new_n279));
  nano32aa1n03x7               g184(.a(new_n224), .b(new_n279), .c(new_n240), .d(new_n263), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n197), .c(new_n129), .d(new_n188), .o1(new_n281));
  nano22aa1n02x4               g186(.a(new_n226), .b(new_n240), .c(new_n263), .out0(new_n282));
  orn002aa1n02x5               g187(.a(\a[25] ), .b(\b[24] ), .o(new_n283));
  oaoi03aa1n02x5               g188(.a(\a[26] ), .b(\b[25] ), .c(new_n283), .o1(new_n284));
  oaoi13aa1n06x5               g189(.a(new_n284), .b(new_n279), .c(new_n282), .d(new_n267), .o1(new_n285));
  xorc02aa1n12x5               g190(.a(\a[27] ), .b(\b[26] ), .out0(new_n286));
  xnbna2aa1n03x5               g191(.a(new_n286), .b(new_n281), .c(new_n285), .out0(\s[27] ));
  nand22aa1n03x5               g192(.a(new_n281), .b(new_n285), .o1(new_n288));
  norp02aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  nor022aa1n08x5               g194(.a(\b[27] ), .b(\a[28] ), .o1(new_n290));
  nand02aa1n06x5               g195(.a(\b[27] ), .b(\a[28] ), .o1(new_n291));
  nanb02aa1n06x5               g196(.a(new_n290), .b(new_n291), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n289), .c(new_n288), .d(new_n286), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n279), .o1(new_n294));
  oabi12aa1n09x5               g199(.a(new_n284), .b(new_n268), .c(new_n294), .out0(new_n295));
  aoai13aa1n04x5               g200(.a(new_n286), .b(new_n295), .c(new_n218), .d(new_n280), .o1(new_n296));
  nona22aa1n03x5               g201(.a(new_n296), .b(new_n292), .c(new_n289), .out0(new_n297));
  nanp02aa1n03x5               g202(.a(new_n293), .b(new_n297), .o1(\s[28] ));
  norb02aa1n03x5               g203(.a(new_n286), .b(new_n292), .out0(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n295), .c(new_n218), .d(new_n280), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n299), .o1(new_n301));
  oai012aa1n02x5               g206(.a(new_n291), .b(new_n290), .c(new_n289), .o1(new_n302));
  aoai13aa1n02x5               g207(.a(new_n302), .b(new_n301), .c(new_n281), .d(new_n285), .o1(new_n303));
  norp02aa1n02x5               g208(.a(\b[28] ), .b(\a[29] ), .o1(new_n304));
  nand42aa1n03x5               g209(.a(\b[28] ), .b(\a[29] ), .o1(new_n305));
  norb02aa1n02x7               g210(.a(new_n305), .b(new_n304), .out0(new_n306));
  oai022aa1n02x5               g211(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n307));
  aboi22aa1n03x5               g212(.a(new_n304), .b(new_n305), .c(new_n307), .d(new_n291), .out0(new_n308));
  aoi022aa1n03x5               g213(.a(new_n303), .b(new_n306), .c(new_n300), .d(new_n308), .o1(\s[29] ));
  xorb03aa1n02x5               g214(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g215(.a(new_n292), .b(new_n286), .c(new_n306), .out0(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n295), .c(new_n218), .d(new_n280), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n311), .o1(new_n313));
  aoi013aa1n02x4               g218(.a(new_n304), .b(new_n307), .c(new_n291), .d(new_n305), .o1(new_n314));
  aoai13aa1n02x5               g219(.a(new_n314), .b(new_n313), .c(new_n281), .d(new_n285), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[30] ), .b(\b[29] ), .out0(new_n316));
  aoi113aa1n02x5               g221(.a(new_n316), .b(new_n304), .c(new_n307), .d(new_n305), .e(new_n291), .o1(new_n317));
  aoi022aa1n03x5               g222(.a(new_n315), .b(new_n316), .c(new_n312), .d(new_n317), .o1(\s[30] ));
  nand03aa1n02x5               g223(.a(new_n299), .b(new_n306), .c(new_n316), .o1(new_n319));
  nanb02aa1n02x5               g224(.a(new_n319), .b(new_n288), .out0(new_n320));
  xorc02aa1n02x5               g225(.a(\a[31] ), .b(\b[30] ), .out0(new_n321));
  oao003aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .c(new_n314), .carry(new_n322));
  norb02aa1n02x5               g227(.a(new_n322), .b(new_n321), .out0(new_n323));
  aoai13aa1n06x5               g228(.a(new_n322), .b(new_n319), .c(new_n281), .d(new_n285), .o1(new_n324));
  aoi022aa1n03x5               g229(.a(new_n320), .b(new_n323), .c(new_n324), .d(new_n321), .o1(\s[31] ));
  xnrb03aa1n02x5               g230(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi122aa1n02x5               g231(.a(new_n106), .b(new_n109), .c(new_n105), .d(new_n103), .e(new_n107), .o1(new_n327));
  oab012aa1n02x4               g232(.a(new_n327), .b(new_n111), .c(new_n104), .out0(\s[4] ));
  xnrc02aa1n02x5               g233(.a(new_n111), .b(new_n112), .out0(\s[5] ));
  oaoi13aa1n04x5               g234(.a(new_n125), .b(new_n112), .c(new_n150), .d(new_n110), .o1(new_n330));
  xnrc02aa1n02x5               g235(.a(new_n330), .b(new_n115), .out0(\s[6] ));
  aob012aa1n02x5               g236(.a(new_n114), .b(new_n330), .c(new_n115), .out0(new_n332));
  and002aa1n02x5               g237(.a(new_n330), .b(new_n115), .o(new_n333));
  nano32aa1n03x7               g238(.a(new_n333), .b(new_n117), .c(new_n124), .d(new_n114), .out0(new_n334));
  aoi012aa1n02x5               g239(.a(new_n334), .b(new_n151), .c(new_n332), .o1(\s[7] ));
  oabi12aa1n02x5               g240(.a(new_n152), .b(new_n334), .c(new_n116), .out0(new_n336));
  nano22aa1n02x4               g241(.a(new_n334), .b(new_n124), .c(new_n152), .out0(new_n337));
  nanb02aa1n02x5               g242(.a(new_n337), .b(new_n336), .out0(\s[8] ));
  xorb03aa1n02x5               g243(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


