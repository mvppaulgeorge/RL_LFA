// Benchmark "adder" written by ABC on Thu Jul 18 06:05:16 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n139, new_n140, new_n141, new_n143, new_n144, new_n145, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n335, new_n338, new_n340, new_n342;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand22aa1n06x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nor002aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  oai012aa1n04x7               g005(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n101));
  xorc02aa1n02x5               g006(.a(\a[4] ), .b(\b[3] ), .out0(new_n102));
  nor042aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  norb02aa1n02x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nanb03aa1n02x5               g010(.a(new_n101), .b(new_n102), .c(new_n105), .out0(new_n106));
  aoi112aa1n02x7               g011(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n107));
  oab012aa1n04x5               g012(.a(new_n107), .b(\a[4] ), .c(\b[3] ), .out0(new_n108));
  nor022aa1n06x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  norp02aa1n06x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nano23aa1n02x4               g017(.a(new_n112), .b(new_n109), .c(new_n110), .d(new_n111), .out0(new_n113));
  nor042aa1n03x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nand02aa1n03x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  norb02aa1n09x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  xorc02aa1n02x5               g021(.a(\a[6] ), .b(\b[5] ), .out0(new_n117));
  nanp03aa1n02x5               g022(.a(new_n113), .b(new_n116), .c(new_n117), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[6] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[5] ), .o1(new_n120));
  oao003aa1n02x5               g025(.a(new_n119), .b(new_n120), .c(new_n114), .carry(new_n121));
  tech160nm_fiao0012aa1n02p5x5 g026(.a(new_n109), .b(new_n112), .c(new_n110), .o(new_n122));
  aoi012aa1n02x5               g027(.a(new_n122), .b(new_n113), .c(new_n121), .o1(new_n123));
  aoai13aa1n04x5               g028(.a(new_n123), .b(new_n118), .c(new_n106), .d(new_n108), .o1(new_n124));
  nanp02aa1n04x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  nor002aa1d24x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand02aa1n08x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nanb02aa1n02x5               g032(.a(new_n126), .b(new_n127), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n97), .c(new_n124), .d(new_n125), .o1(new_n129));
  tech160nm_fixnrc02aa1n02p5x5 g034(.a(\b[3] ), .b(\a[4] ), .out0(new_n130));
  nanb02aa1n02x5               g035(.a(new_n103), .b(new_n104), .out0(new_n131));
  oai013aa1n06x5               g036(.a(new_n108), .b(new_n130), .c(new_n101), .d(new_n131), .o1(new_n132));
  nona23aa1n02x4               g037(.a(new_n111), .b(new_n110), .c(new_n112), .d(new_n109), .out0(new_n133));
  inv000aa1d42x5               g038(.a(new_n116), .o1(new_n134));
  xnrc02aa1n02x5               g039(.a(\b[5] ), .b(\a[6] ), .out0(new_n135));
  nor043aa1n03x5               g040(.a(new_n133), .b(new_n134), .c(new_n135), .o1(new_n136));
  oaoi03aa1n02x5               g041(.a(new_n119), .b(new_n120), .c(new_n114), .o1(new_n137));
  oabi12aa1n06x5               g042(.a(new_n122), .b(new_n133), .c(new_n137), .out0(new_n138));
  norb02aa1n02x5               g043(.a(new_n125), .b(new_n97), .out0(new_n139));
  aoai13aa1n02x5               g044(.a(new_n139), .b(new_n138), .c(new_n132), .d(new_n136), .o1(new_n140));
  nona22aa1n02x4               g045(.a(new_n140), .b(new_n128), .c(new_n97), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(new_n129), .b(new_n141), .o1(\s[10] ));
  nor002aa1d32x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  nand22aa1n12x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  xobna2aa1n03x5               g050(.a(new_n145), .b(new_n141), .c(new_n127), .out0(\s[11] ));
  norp02aa1n24x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nand02aa1d28x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  aoi113aa1n02x5               g054(.a(new_n149), .b(new_n143), .c(new_n141), .d(new_n145), .e(new_n127), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n143), .o1(new_n151));
  nanp03aa1n02x5               g056(.a(new_n141), .b(new_n127), .c(new_n145), .o1(new_n152));
  aobi12aa1n02x5               g057(.a(new_n149), .b(new_n152), .c(new_n151), .out0(new_n153));
  norp02aa1n02x5               g058(.a(new_n153), .b(new_n150), .o1(\s[12] ));
  nano23aa1n06x5               g059(.a(new_n143), .b(new_n147), .c(new_n148), .d(new_n144), .out0(new_n155));
  nano23aa1n09x5               g060(.a(new_n97), .b(new_n126), .c(new_n127), .d(new_n125), .out0(new_n156));
  nand22aa1n09x5               g061(.a(new_n156), .b(new_n155), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n138), .c(new_n132), .d(new_n136), .o1(new_n159));
  nona23aa1n09x5               g064(.a(new_n148), .b(new_n144), .c(new_n143), .d(new_n147), .out0(new_n160));
  oaih12aa1n12x5               g065(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n161));
  aoi012aa1d18x5               g066(.a(new_n147), .b(new_n143), .c(new_n148), .o1(new_n162));
  oai012aa1d24x5               g067(.a(new_n162), .b(new_n160), .c(new_n161), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  nor022aa1n08x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n03x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n159), .c(new_n164), .out0(\s[13] ));
  orn002aa1n02x5               g073(.a(\a[13] ), .b(\b[12] ), .o(new_n169));
  aoai13aa1n02x5               g074(.a(new_n167), .b(new_n163), .c(new_n124), .d(new_n158), .o1(new_n170));
  norp02aa1n03x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nand42aa1n02x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  norb02aa1n02x7               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n170), .c(new_n169), .out0(\s[14] ));
  nona23aa1n03x5               g079(.a(new_n172), .b(new_n166), .c(new_n165), .d(new_n171), .out0(new_n175));
  aoi012aa1n02x5               g080(.a(new_n171), .b(new_n165), .c(new_n172), .o1(new_n176));
  aoai13aa1n03x5               g081(.a(new_n176), .b(new_n175), .c(new_n159), .d(new_n164), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nand22aa1n03x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nor022aa1n03x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nand42aa1n02x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nanb02aa1n02x5               g087(.a(new_n181), .b(new_n182), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  aoi112aa1n02x5               g089(.a(new_n184), .b(new_n179), .c(new_n177), .d(new_n180), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n184), .b(new_n179), .c(new_n177), .d(new_n180), .o1(new_n186));
  norb02aa1n03x4               g091(.a(new_n186), .b(new_n185), .out0(\s[16] ));
  nano23aa1n02x4               g092(.a(new_n179), .b(new_n181), .c(new_n182), .d(new_n180), .out0(new_n188));
  nano32aa1n03x7               g093(.a(new_n157), .b(new_n188), .c(new_n167), .d(new_n173), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n138), .c(new_n132), .d(new_n136), .o1(new_n190));
  nona23aa1n03x5               g095(.a(new_n182), .b(new_n180), .c(new_n179), .d(new_n181), .out0(new_n191));
  norp02aa1n03x5               g096(.a(new_n191), .b(new_n175), .o1(new_n192));
  norp02aa1n02x5               g097(.a(new_n191), .b(new_n176), .o1(new_n193));
  oa0012aa1n02x5               g098(.a(new_n182), .b(new_n181), .c(new_n179), .o(new_n194));
  aoi112aa1n06x5               g099(.a(new_n194), .b(new_n193), .c(new_n163), .d(new_n192), .o1(new_n195));
  xorc02aa1n02x5               g100(.a(\a[17] ), .b(\b[16] ), .out0(new_n196));
  xnbna2aa1n03x5               g101(.a(new_n196), .b(new_n190), .c(new_n195), .out0(\s[17] ));
  inv040aa1d32x5               g102(.a(\a[18] ), .o1(new_n198));
  nanp02aa1n06x5               g103(.a(new_n190), .b(new_n195), .o1(new_n199));
  norp02aa1n02x5               g104(.a(\b[16] ), .b(\a[17] ), .o1(new_n200));
  tech160nm_fiaoi012aa1n05x5   g105(.a(new_n200), .b(new_n199), .c(new_n196), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[17] ), .c(new_n198), .out0(\s[18] ));
  inv000aa1d42x5               g107(.a(\a[17] ), .o1(new_n203));
  xroi22aa1d06x4               g108(.a(new_n203), .b(\b[16] ), .c(new_n198), .d(\b[17] ), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  inv000aa1d42x5               g110(.a(\b[17] ), .o1(new_n206));
  oao003aa1n02x5               g111(.a(new_n198), .b(new_n206), .c(new_n200), .carry(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n04x5               g113(.a(new_n208), .b(new_n205), .c(new_n190), .d(new_n195), .o1(new_n209));
  xorb03aa1n02x5               g114(.a(new_n209), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nanp02aa1n02x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nor042aa1n04x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  tech160nm_finand02aa1n03p5x5 g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  norb02aa1n12x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  aoi112aa1n02x5               g121(.a(new_n212), .b(new_n216), .c(new_n209), .d(new_n213), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n212), .o1(new_n218));
  norb02aa1n06x4               g123(.a(new_n213), .b(new_n212), .out0(new_n219));
  nand42aa1n02x5               g124(.a(new_n209), .b(new_n219), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n216), .o1(new_n221));
  aoi012aa1n06x5               g126(.a(new_n221), .b(new_n220), .c(new_n218), .o1(new_n222));
  norp02aa1n03x5               g127(.a(new_n222), .b(new_n217), .o1(\s[20] ));
  nano23aa1n02x4               g128(.a(new_n212), .b(new_n214), .c(new_n215), .d(new_n213), .out0(new_n224));
  nanp02aa1n02x5               g129(.a(new_n204), .b(new_n224), .o1(new_n225));
  aoi112aa1n02x5               g130(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n226));
  norp02aa1n02x5               g131(.a(\b[17] ), .b(\a[18] ), .o1(new_n227));
  aoi112aa1n09x5               g132(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n228));
  oai112aa1n06x5               g133(.a(new_n219), .b(new_n216), .c(new_n228), .d(new_n227), .o1(new_n229));
  nona22aa1d18x5               g134(.a(new_n229), .b(new_n226), .c(new_n214), .out0(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  aoai13aa1n06x5               g136(.a(new_n231), .b(new_n225), .c(new_n190), .d(new_n195), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  xnrc02aa1n12x5               g139(.a(\b[20] ), .b(\a[21] ), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  xnrc02aa1n12x5               g141(.a(\b[21] ), .b(\a[22] ), .out0(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  aoi112aa1n02x7               g143(.a(new_n234), .b(new_n238), .c(new_n232), .d(new_n236), .o1(new_n239));
  inv000aa1n02x5               g144(.a(new_n234), .o1(new_n240));
  nand42aa1n02x5               g145(.a(new_n232), .b(new_n236), .o1(new_n241));
  tech160nm_fiaoi012aa1n02p5x5 g146(.a(new_n237), .b(new_n241), .c(new_n240), .o1(new_n242));
  norp02aa1n03x5               g147(.a(new_n242), .b(new_n239), .o1(\s[22] ));
  nor022aa1n04x5               g148(.a(new_n237), .b(new_n235), .o1(new_n244));
  nand23aa1n03x5               g149(.a(new_n204), .b(new_n244), .c(new_n224), .o1(new_n245));
  oaoi03aa1n02x5               g150(.a(\a[22] ), .b(\b[21] ), .c(new_n240), .o1(new_n246));
  tech160nm_fiaoi012aa1n02p5x5 g151(.a(new_n246), .b(new_n230), .c(new_n244), .o1(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n245), .c(new_n190), .d(new_n195), .o1(new_n248));
  xorb03aa1n02x5               g153(.a(new_n248), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  nand42aa1n02x5               g155(.a(\b[22] ), .b(\a[23] ), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n251), .b(new_n250), .out0(new_n252));
  norp02aa1n04x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  nanp02aa1n02x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  norb02aa1n09x5               g159(.a(new_n254), .b(new_n253), .out0(new_n255));
  aoi112aa1n02x5               g160(.a(new_n250), .b(new_n255), .c(new_n248), .d(new_n252), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n250), .o1(new_n257));
  nand42aa1n02x5               g162(.a(new_n248), .b(new_n252), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n255), .o1(new_n259));
  tech160nm_fiaoi012aa1n05x5   g164(.a(new_n259), .b(new_n258), .c(new_n257), .o1(new_n260));
  nor002aa1n02x5               g165(.a(new_n260), .b(new_n256), .o1(\s[24] ));
  nona23aa1n09x5               g166(.a(new_n254), .b(new_n251), .c(new_n250), .d(new_n253), .out0(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  nanb03aa1n03x5               g168(.a(new_n225), .b(new_n263), .c(new_n244), .out0(new_n264));
  aoi112aa1n02x5               g169(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n265));
  aoi113aa1n03x7               g170(.a(new_n265), .b(new_n253), .c(new_n246), .d(new_n255), .e(new_n252), .o1(new_n266));
  inv020aa1n03x5               g171(.a(new_n266), .o1(new_n267));
  aoi013aa1n02x4               g172(.a(new_n267), .b(new_n230), .c(new_n244), .d(new_n263), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n264), .c(new_n190), .d(new_n195), .o1(new_n269));
  xorb03aa1n02x5               g174(.a(new_n269), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g175(.a(\b[24] ), .b(\a[25] ), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[25] ), .b(\b[24] ), .out0(new_n272));
  xorc02aa1n12x5               g177(.a(\a[26] ), .b(\b[25] ), .out0(new_n273));
  aoi112aa1n03x4               g178(.a(new_n271), .b(new_n273), .c(new_n269), .d(new_n272), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n271), .o1(new_n275));
  nand02aa1n02x5               g180(.a(new_n269), .b(new_n272), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n273), .o1(new_n277));
  aoi012aa1n06x5               g182(.a(new_n277), .b(new_n276), .c(new_n275), .o1(new_n278));
  nor042aa1n03x5               g183(.a(new_n278), .b(new_n274), .o1(\s[26] ));
  inv000aa1d42x5               g184(.a(new_n161), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n162), .o1(new_n281));
  aoai13aa1n06x5               g186(.a(new_n192), .b(new_n281), .c(new_n155), .d(new_n280), .o1(new_n282));
  nona22aa1n02x4               g187(.a(new_n282), .b(new_n193), .c(new_n194), .out0(new_n283));
  and002aa1n18x5               g188(.a(new_n273), .b(new_n272), .o(new_n284));
  nano22aa1n03x7               g189(.a(new_n245), .b(new_n284), .c(new_n263), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n285), .b(new_n283), .c(new_n124), .d(new_n189), .o1(new_n286));
  nona32aa1n09x5               g191(.a(new_n230), .b(new_n262), .c(new_n237), .d(new_n235), .out0(new_n287));
  nand22aa1n03x5               g192(.a(new_n287), .b(new_n266), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(\b[25] ), .b(\a[26] ), .o1(new_n289));
  oai022aa1n02x5               g194(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n290));
  aoi022aa1n09x5               g195(.a(new_n288), .b(new_n284), .c(new_n289), .d(new_n290), .o1(new_n291));
  xorc02aa1n12x5               g196(.a(\a[27] ), .b(\b[26] ), .out0(new_n292));
  xnbna2aa1n03x5               g197(.a(new_n292), .b(new_n291), .c(new_n286), .out0(\s[27] ));
  norp02aa1n02x5               g198(.a(\b[26] ), .b(\a[27] ), .o1(new_n294));
  inv040aa1n03x5               g199(.a(new_n294), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n292), .o1(new_n296));
  aoi012aa1n02x7               g201(.a(new_n296), .b(new_n291), .c(new_n286), .o1(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[27] ), .b(\a[28] ), .out0(new_n298));
  nano22aa1n03x5               g203(.a(new_n297), .b(new_n295), .c(new_n298), .out0(new_n299));
  inv000aa1d42x5               g204(.a(new_n284), .o1(new_n300));
  nanp02aa1n02x5               g205(.a(new_n290), .b(new_n289), .o1(new_n301));
  aoai13aa1n06x5               g206(.a(new_n301), .b(new_n300), .c(new_n287), .d(new_n266), .o1(new_n302));
  aoai13aa1n03x5               g207(.a(new_n292), .b(new_n302), .c(new_n199), .d(new_n285), .o1(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n298), .b(new_n303), .c(new_n295), .o1(new_n304));
  nor002aa1n02x5               g209(.a(new_n304), .b(new_n299), .o1(\s[28] ));
  xnrc02aa1n02x5               g210(.a(\b[28] ), .b(\a[29] ), .out0(new_n306));
  norb02aa1n02x5               g211(.a(new_n292), .b(new_n298), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n302), .c(new_n199), .d(new_n285), .o1(new_n308));
  oao003aa1n02x5               g213(.a(\a[28] ), .b(\b[27] ), .c(new_n295), .carry(new_n309));
  tech160nm_fiaoi012aa1n02p5x5 g214(.a(new_n306), .b(new_n308), .c(new_n309), .o1(new_n310));
  inv000aa1n02x5               g215(.a(new_n307), .o1(new_n311));
  aoi012aa1n02x7               g216(.a(new_n311), .b(new_n291), .c(new_n286), .o1(new_n312));
  nano22aa1n03x5               g217(.a(new_n312), .b(new_n306), .c(new_n309), .out0(new_n313));
  norp02aa1n03x5               g218(.a(new_n310), .b(new_n313), .o1(\s[29] ));
  xorb03aa1n02x5               g219(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n03x5               g220(.a(new_n292), .b(new_n306), .c(new_n298), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n302), .c(new_n199), .d(new_n285), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[29] ), .b(\b[28] ), .c(new_n309), .carry(new_n318));
  xnrc02aa1n02x5               g223(.a(\b[29] ), .b(\a[30] ), .out0(new_n319));
  tech160nm_fiaoi012aa1n02p5x5 g224(.a(new_n319), .b(new_n317), .c(new_n318), .o1(new_n320));
  inv000aa1d42x5               g225(.a(new_n316), .o1(new_n321));
  aoi012aa1n02x7               g226(.a(new_n321), .b(new_n291), .c(new_n286), .o1(new_n322));
  nano22aa1n03x5               g227(.a(new_n322), .b(new_n318), .c(new_n319), .out0(new_n323));
  norp02aa1n03x5               g228(.a(new_n320), .b(new_n323), .o1(\s[30] ));
  norb02aa1n06x5               g229(.a(new_n316), .b(new_n319), .out0(new_n325));
  inv000aa1n02x5               g230(.a(new_n325), .o1(new_n326));
  aoi012aa1n02x7               g231(.a(new_n326), .b(new_n291), .c(new_n286), .o1(new_n327));
  oao003aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .c(new_n318), .carry(new_n328));
  xnrc02aa1n02x5               g233(.a(\b[30] ), .b(\a[31] ), .out0(new_n329));
  nano22aa1n03x5               g234(.a(new_n327), .b(new_n328), .c(new_n329), .out0(new_n330));
  aoai13aa1n03x5               g235(.a(new_n325), .b(new_n302), .c(new_n199), .d(new_n285), .o1(new_n331));
  tech160nm_fiaoi012aa1n02p5x5 g236(.a(new_n329), .b(new_n331), .c(new_n328), .o1(new_n332));
  norp02aa1n03x5               g237(.a(new_n332), .b(new_n330), .o1(\s[31] ));
  xnrb03aa1n02x5               g238(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g239(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n335));
  xorb03aa1n02x5               g240(.a(new_n335), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g241(.a(new_n132), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g242(.a(new_n115), .b(new_n132), .c(new_n114), .o1(new_n338));
  xorb03aa1n02x5               g243(.a(new_n338), .b(\b[5] ), .c(new_n119), .out0(\s[6] ));
  oaoi03aa1n02x5               g244(.a(\a[6] ), .b(\b[5] ), .c(new_n338), .o1(new_n340));
  xorb03aa1n02x5               g245(.a(new_n340), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oai012aa1n02x5               g246(.a(new_n111), .b(new_n340), .c(new_n112), .o1(new_n342));
  xnrb03aa1n02x5               g247(.a(new_n342), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g248(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


