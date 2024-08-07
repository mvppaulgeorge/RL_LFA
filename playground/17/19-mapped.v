// Benchmark "adder" written by ABC on Wed Jul 17 20:49:24 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n328, new_n331, new_n333, new_n334,
    new_n335;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nanp02aa1n06x5               g002(.a(\b[3] ), .b(\a[4] ), .o1(new_n98));
  nand22aa1n04x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nand02aa1n06x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nona22aa1n09x5               g006(.a(new_n100), .b(new_n101), .c(new_n99), .out0(new_n102));
  nor042aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nano22aa1n03x7               g009(.a(new_n103), .b(new_n100), .c(new_n104), .out0(new_n105));
  orn002aa1n24x5               g010(.a(\a[4] ), .b(\b[3] ), .o(new_n106));
  oai112aa1n06x5               g011(.a(new_n106), .b(new_n98), .c(\b[2] ), .d(\a[3] ), .o1(new_n107));
  aoai13aa1n04x5               g012(.a(new_n98), .b(new_n107), .c(new_n105), .d(new_n102), .o1(new_n108));
  nor042aa1n03x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nand42aa1n04x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor042aa1n09x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n04x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nano23aa1n06x5               g017(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n113));
  xorc02aa1n02x5               g018(.a(\a[6] ), .b(\b[5] ), .out0(new_n114));
  tech160nm_fixnrc02aa1n05x5   g019(.a(\b[4] ), .b(\a[5] ), .out0(new_n115));
  inv030aa1n02x5               g020(.a(new_n115), .o1(new_n116));
  nand03aa1n02x5               g021(.a(new_n116), .b(new_n113), .c(new_n114), .o1(new_n117));
  inv000aa1n02x5               g022(.a(new_n111), .o1(new_n118));
  oaoi03aa1n02x5               g023(.a(\a[8] ), .b(\b[7] ), .c(new_n118), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\a[6] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[5] ), .o1(new_n121));
  nor042aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  oao003aa1n02x5               g027(.a(new_n120), .b(new_n121), .c(new_n122), .carry(new_n123));
  aoi012aa1n06x5               g028(.a(new_n119), .b(new_n113), .c(new_n123), .o1(new_n124));
  tech160nm_fioai012aa1n05x5   g029(.a(new_n124), .b(new_n108), .c(new_n117), .o1(new_n125));
  nand02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  aoi012aa1n02x5               g031(.a(new_n97), .b(new_n125), .c(new_n126), .o1(new_n127));
  xnrb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n04x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand22aa1n09x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nano23aa1n06x5               g035(.a(new_n97), .b(new_n129), .c(new_n130), .d(new_n126), .out0(new_n131));
  oai012aa1n18x5               g036(.a(new_n130), .b(new_n129), .c(new_n97), .o1(new_n132));
  inv040aa1n02x5               g037(.a(new_n132), .o1(new_n133));
  nor042aa1n06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand02aa1d04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n133), .c(new_n125), .d(new_n131), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n136), .b(new_n133), .c(new_n125), .d(new_n131), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  inv000aa1n06x5               g044(.a(new_n134), .o1(new_n140));
  nor042aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1n03x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanb02aa1n02x5               g047(.a(new_n141), .b(new_n142), .out0(new_n143));
  xobna2aa1n03x5               g048(.a(new_n143), .b(new_n137), .c(new_n140), .out0(\s[12] ));
  norb02aa1n03x5               g049(.a(new_n104), .b(new_n103), .out0(new_n145));
  oai112aa1n03x5               g050(.a(new_n145), .b(new_n100), .c(new_n101), .d(new_n99), .o1(new_n146));
  inv040aa1n03x5               g051(.a(new_n107), .o1(new_n147));
  aobi12aa1n06x5               g052(.a(new_n98), .b(new_n146), .c(new_n147), .out0(new_n148));
  norb02aa1n02x7               g053(.a(new_n110), .b(new_n109), .out0(new_n149));
  norb02aa1n02x5               g054(.a(new_n112), .b(new_n111), .out0(new_n150));
  nano32aa1n03x7               g055(.a(new_n115), .b(new_n114), .c(new_n150), .d(new_n149), .out0(new_n151));
  inv020aa1n02x5               g056(.a(new_n124), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n152), .b(new_n148), .c(new_n151), .o1(new_n153));
  nano23aa1n06x5               g058(.a(new_n134), .b(new_n141), .c(new_n142), .d(new_n135), .out0(new_n154));
  nand02aa1n02x5               g059(.a(new_n154), .b(new_n131), .o1(new_n155));
  nona23aa1n09x5               g060(.a(new_n142), .b(new_n135), .c(new_n134), .d(new_n141), .out0(new_n156));
  oaoi03aa1n12x5               g061(.a(\a[12] ), .b(\b[11] ), .c(new_n140), .o1(new_n157));
  inv000aa1n02x5               g062(.a(new_n157), .o1(new_n158));
  oai012aa1n03x5               g063(.a(new_n158), .b(new_n156), .c(new_n132), .o1(new_n159));
  oabi12aa1n02x5               g064(.a(new_n159), .b(new_n153), .c(new_n155), .out0(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n08x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nand02aa1n06x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n162), .b(new_n160), .c(new_n163), .o1(new_n164));
  xnrb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n16x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nand02aa1n04x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nona23aa1d18x5               g072(.a(new_n167), .b(new_n163), .c(new_n162), .d(new_n166), .out0(new_n168));
  inv040aa1n08x5               g073(.a(new_n168), .o1(new_n169));
  oai012aa1n02x5               g074(.a(new_n167), .b(new_n166), .c(new_n162), .o1(new_n170));
  inv000aa1n02x5               g075(.a(new_n170), .o1(new_n171));
  tech160nm_fiaoi012aa1n04x5   g076(.a(new_n171), .b(new_n159), .c(new_n169), .o1(new_n172));
  nona22aa1n03x5               g077(.a(new_n125), .b(new_n155), .c(new_n168), .out0(new_n173));
  nor042aa1n04x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nand42aa1n04x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n173), .c(new_n172), .out0(\s[15] ));
  aobi12aa1n06x5               g082(.a(new_n176), .b(new_n173), .c(new_n172), .out0(new_n178));
  nor002aa1n03x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nand02aa1n08x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  obai22aa1n02x7               g085(.a(new_n180), .b(new_n179), .c(new_n178), .d(new_n174), .out0(new_n181));
  nona32aa1n02x4               g086(.a(new_n180), .b(new_n178), .c(new_n179), .d(new_n174), .out0(new_n182));
  nanp02aa1n02x5               g087(.a(new_n182), .b(new_n181), .o1(\s[16] ));
  nano23aa1n06x5               g088(.a(new_n174), .b(new_n179), .c(new_n180), .d(new_n175), .out0(new_n184));
  inv000aa1n04x5               g089(.a(new_n184), .o1(new_n185));
  nor043aa1n03x5               g090(.a(new_n155), .b(new_n168), .c(new_n185), .o1(new_n186));
  aoai13aa1n12x5               g091(.a(new_n186), .b(new_n152), .c(new_n148), .d(new_n151), .o1(new_n187));
  oaoi13aa1n06x5               g092(.a(new_n168), .b(new_n158), .c(new_n156), .d(new_n132), .o1(new_n188));
  oa0012aa1n06x5               g093(.a(new_n180), .b(new_n179), .c(new_n174), .o(new_n189));
  oaoi13aa1n09x5               g094(.a(new_n189), .b(new_n184), .c(new_n188), .d(new_n171), .o1(new_n190));
  xorc02aa1n02x5               g095(.a(\a[17] ), .b(\b[16] ), .out0(new_n191));
  xnbna2aa1n03x5               g096(.a(new_n191), .b(new_n190), .c(new_n187), .out0(\s[17] ));
  inv000aa1d42x5               g097(.a(\a[17] ), .o1(new_n193));
  nanb02aa1n02x5               g098(.a(\b[16] ), .b(new_n193), .out0(new_n194));
  nona23aa1n09x5               g099(.a(new_n184), .b(new_n131), .c(new_n156), .d(new_n168), .out0(new_n195));
  oaoi13aa1n12x5               g100(.a(new_n195), .b(new_n124), .c(new_n108), .d(new_n117), .o1(new_n196));
  aoai13aa1n04x5               g101(.a(new_n169), .b(new_n157), .c(new_n154), .d(new_n133), .o1(new_n197));
  inv000aa1n02x5               g102(.a(new_n189), .o1(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n185), .c(new_n197), .d(new_n170), .o1(new_n199));
  oaih12aa1n02x5               g104(.a(new_n191), .b(new_n199), .c(new_n196), .o1(new_n200));
  xnrc02aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .out0(new_n201));
  xobna2aa1n03x5               g106(.a(new_n201), .b(new_n200), .c(new_n194), .out0(\s[18] ));
  inv040aa1d32x5               g107(.a(\a[18] ), .o1(new_n203));
  xroi22aa1d06x4               g108(.a(new_n193), .b(\b[16] ), .c(new_n203), .d(\b[17] ), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  oai022aa1n02x7               g110(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n206));
  oaib12aa1n09x5               g111(.a(new_n206), .b(new_n203), .c(\b[17] ), .out0(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n205), .c(new_n190), .d(new_n187), .o1(new_n208));
  xorb03aa1n02x5               g113(.a(new_n208), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nand02aa1d06x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nanb02aa1n02x5               g117(.a(new_n211), .b(new_n212), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  norp02aa1n04x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nand02aa1n08x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nanb02aa1n02x5               g121(.a(new_n215), .b(new_n216), .out0(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n211), .c(new_n208), .d(new_n214), .o1(new_n218));
  oai112aa1n06x5               g123(.a(new_n187), .b(new_n198), .c(new_n185), .d(new_n172), .o1(new_n219));
  oaoi03aa1n09x5               g124(.a(\a[18] ), .b(\b[17] ), .c(new_n194), .o1(new_n220));
  aoai13aa1n03x5               g125(.a(new_n214), .b(new_n220), .c(new_n219), .d(new_n204), .o1(new_n221));
  nona22aa1n03x5               g126(.a(new_n221), .b(new_n217), .c(new_n211), .out0(new_n222));
  nanp02aa1n03x5               g127(.a(new_n218), .b(new_n222), .o1(\s[20] ));
  nano23aa1n06x5               g128(.a(new_n211), .b(new_n215), .c(new_n216), .d(new_n212), .out0(new_n224));
  nanb03aa1n02x5               g129(.a(new_n201), .b(new_n224), .c(new_n191), .out0(new_n225));
  nona23aa1n09x5               g130(.a(new_n216), .b(new_n212), .c(new_n211), .d(new_n215), .out0(new_n226));
  oa0012aa1n06x5               g131(.a(new_n216), .b(new_n215), .c(new_n211), .o(new_n227));
  inv040aa1n03x5               g132(.a(new_n227), .o1(new_n228));
  oai012aa1n18x5               g133(.a(new_n228), .b(new_n226), .c(new_n207), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoai13aa1n04x5               g135(.a(new_n230), .b(new_n225), .c(new_n190), .d(new_n187), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[20] ), .b(\a[21] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  tech160nm_fixnrc02aa1n05x5   g140(.a(\b[21] ), .b(\a[22] ), .out0(new_n236));
  aoai13aa1n03x5               g141(.a(new_n236), .b(new_n233), .c(new_n231), .d(new_n235), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(new_n231), .b(new_n235), .o1(new_n238));
  nona22aa1n02x4               g143(.a(new_n238), .b(new_n236), .c(new_n233), .out0(new_n239));
  nanp02aa1n03x5               g144(.a(new_n239), .b(new_n237), .o1(\s[22] ));
  nor042aa1n06x5               g145(.a(new_n236), .b(new_n234), .o1(new_n241));
  nand23aa1d12x5               g146(.a(new_n204), .b(new_n241), .c(new_n224), .o1(new_n242));
  inv000aa1d42x5               g147(.a(\a[22] ), .o1(new_n243));
  inv000aa1d42x5               g148(.a(\b[21] ), .o1(new_n244));
  oao003aa1n02x5               g149(.a(new_n243), .b(new_n244), .c(new_n233), .carry(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n229), .c(new_n241), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n242), .c(new_n190), .d(new_n187), .o1(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n04x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  xorc02aa1n12x5               g154(.a(\a[23] ), .b(\b[22] ), .out0(new_n250));
  tech160nm_fixnrc02aa1n05x5   g155(.a(\b[23] ), .b(\a[24] ), .out0(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n249), .c(new_n247), .d(new_n250), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(new_n247), .b(new_n250), .o1(new_n253));
  nona22aa1n02x4               g158(.a(new_n253), .b(new_n251), .c(new_n249), .out0(new_n254));
  nanp02aa1n03x5               g159(.a(new_n254), .b(new_n252), .o1(\s[24] ));
  norb02aa1n02x5               g160(.a(new_n250), .b(new_n251), .out0(new_n256));
  nanb03aa1n02x5               g161(.a(new_n225), .b(new_n256), .c(new_n241), .out0(new_n257));
  aoai13aa1n06x5               g162(.a(new_n241), .b(new_n227), .c(new_n224), .d(new_n220), .o1(new_n258));
  inv000aa1n02x5               g163(.a(new_n245), .o1(new_n259));
  inv000aa1n02x5               g164(.a(new_n256), .o1(new_n260));
  inv000aa1d42x5               g165(.a(\a[24] ), .o1(new_n261));
  inv000aa1d42x5               g166(.a(\b[23] ), .o1(new_n262));
  oao003aa1n02x5               g167(.a(new_n261), .b(new_n262), .c(new_n249), .carry(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n260), .c(new_n258), .d(new_n259), .o1(new_n265));
  inv000aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  aoai13aa1n04x5               g171(.a(new_n266), .b(new_n257), .c(new_n190), .d(new_n187), .o1(new_n267));
  xorb03aa1n02x5               g172(.a(new_n267), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  xorc02aa1n12x5               g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  tech160nm_fixnrc02aa1n04x5   g175(.a(\b[25] ), .b(\a[26] ), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n269), .c(new_n267), .d(new_n270), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(new_n267), .b(new_n270), .o1(new_n273));
  nona22aa1n02x4               g178(.a(new_n273), .b(new_n271), .c(new_n269), .out0(new_n274));
  nanp02aa1n03x5               g179(.a(new_n274), .b(new_n272), .o1(\s[26] ));
  norb02aa1n06x5               g180(.a(new_n270), .b(new_n271), .out0(new_n276));
  nano22aa1d15x5               g181(.a(new_n242), .b(new_n256), .c(new_n276), .out0(new_n277));
  oai012aa1d24x5               g182(.a(new_n277), .b(new_n199), .c(new_n196), .o1(new_n278));
  nanp02aa1n02x5               g183(.a(\b[25] ), .b(\a[26] ), .o1(new_n279));
  oai022aa1n02x5               g184(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n280));
  aoi022aa1n12x5               g185(.a(new_n265), .b(new_n276), .c(new_n279), .d(new_n280), .o1(new_n281));
  xorc02aa1n12x5               g186(.a(\a[27] ), .b(\b[26] ), .out0(new_n282));
  xnbna2aa1n03x5               g187(.a(new_n282), .b(new_n278), .c(new_n281), .out0(\s[27] ));
  nor042aa1n03x5               g188(.a(\b[26] ), .b(\a[27] ), .o1(new_n284));
  inv000aa1n03x5               g189(.a(new_n284), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n282), .o1(new_n286));
  aoai13aa1n04x5               g191(.a(new_n285), .b(new_n286), .c(new_n278), .d(new_n281), .o1(new_n287));
  xnrc02aa1n12x5               g192(.a(\b[27] ), .b(\a[28] ), .out0(new_n288));
  nanp02aa1n03x5               g193(.a(new_n287), .b(new_n288), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n256), .b(new_n245), .c(new_n229), .d(new_n241), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n276), .o1(new_n291));
  nanp02aa1n02x5               g196(.a(new_n280), .b(new_n279), .o1(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n291), .c(new_n290), .d(new_n264), .o1(new_n293));
  aoai13aa1n03x5               g198(.a(new_n282), .b(new_n293), .c(new_n219), .d(new_n277), .o1(new_n294));
  nona22aa1n03x5               g199(.a(new_n294), .b(new_n288), .c(new_n284), .out0(new_n295));
  nanp02aa1n03x5               g200(.a(new_n289), .b(new_n295), .o1(\s[28] ));
  norb02aa1n02x5               g201(.a(new_n282), .b(new_n288), .out0(new_n297));
  inv000aa1n02x5               g202(.a(new_n297), .o1(new_n298));
  xorc02aa1n12x5               g203(.a(\a[29] ), .b(\b[28] ), .out0(new_n299));
  inv000aa1d42x5               g204(.a(new_n299), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[28] ), .b(\b[27] ), .c(new_n285), .carry(new_n301));
  norb02aa1n02x5               g206(.a(new_n301), .b(new_n300), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n298), .c(new_n278), .d(new_n281), .o1(new_n303));
  aoai13aa1n06x5               g208(.a(new_n301), .b(new_n298), .c(new_n278), .d(new_n281), .o1(new_n304));
  nanp02aa1n03x5               g209(.a(new_n304), .b(new_n300), .o1(new_n305));
  nanp02aa1n03x5               g210(.a(new_n305), .b(new_n303), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n02x5               g212(.a(new_n288), .b(new_n299), .c(new_n282), .out0(new_n308));
  oao003aa1n03x5               g213(.a(\a[29] ), .b(\b[28] ), .c(new_n301), .carry(new_n309));
  aoai13aa1n04x5               g214(.a(new_n309), .b(new_n308), .c(new_n278), .d(new_n281), .o1(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[29] ), .b(\a[30] ), .out0(new_n311));
  nanp02aa1n03x5               g216(.a(new_n310), .b(new_n311), .o1(new_n312));
  norb02aa1n02x5               g217(.a(new_n309), .b(new_n311), .out0(new_n313));
  aoai13aa1n02x5               g218(.a(new_n313), .b(new_n308), .c(new_n278), .d(new_n281), .o1(new_n314));
  nanp02aa1n03x5               g219(.a(new_n312), .b(new_n314), .o1(\s[30] ));
  nano23aa1d15x5               g220(.a(new_n311), .b(new_n288), .c(new_n282), .d(new_n299), .out0(new_n316));
  inv000aa1n02x5               g221(.a(new_n316), .o1(new_n317));
  oaoi03aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .c(new_n309), .o1(new_n318));
  inv000aa1n02x5               g223(.a(new_n318), .o1(new_n319));
  aoai13aa1n06x5               g224(.a(new_n319), .b(new_n317), .c(new_n278), .d(new_n281), .o1(new_n320));
  xnrc02aa1n02x5               g225(.a(\b[30] ), .b(\a[31] ), .out0(new_n321));
  nanp02aa1n03x5               g226(.a(new_n320), .b(new_n321), .o1(new_n322));
  aoai13aa1n03x5               g227(.a(new_n316), .b(new_n293), .c(new_n219), .d(new_n277), .o1(new_n323));
  nona22aa1n02x4               g228(.a(new_n323), .b(new_n318), .c(new_n321), .out0(new_n324));
  nanp02aa1n03x5               g229(.a(new_n322), .b(new_n324), .o1(\s[31] ));
  xobna2aa1n03x5               g230(.a(new_n145), .b(new_n102), .c(new_n100), .out0(\s[3] ));
  nanp02aa1n02x5               g231(.a(new_n146), .b(new_n147), .o1(new_n327));
  aoi012aa1n02x5               g232(.a(new_n103), .b(new_n105), .c(new_n102), .o1(new_n328));
  aoai13aa1n02x5               g233(.a(new_n327), .b(new_n328), .c(new_n106), .d(new_n98), .o1(\s[4] ));
  xnrc02aa1n02x5               g234(.a(new_n108), .b(new_n116), .out0(\s[5] ));
  aoi013aa1n02x4               g235(.a(new_n122), .b(new_n327), .c(new_n98), .d(new_n116), .o1(new_n331));
  xorb03aa1n02x5               g236(.a(new_n331), .b(\b[5] ), .c(new_n120), .out0(\s[6] ));
  nanp02aa1n02x5               g237(.a(new_n331), .b(new_n114), .o1(new_n333));
  oai112aa1n02x5               g238(.a(new_n333), .b(new_n150), .c(new_n121), .d(new_n120), .o1(new_n334));
  oaoi13aa1n02x5               g239(.a(new_n150), .b(new_n333), .c(new_n120), .d(new_n121), .o1(new_n335));
  norb02aa1n02x5               g240(.a(new_n334), .b(new_n335), .out0(\s[7] ));
  xnbna2aa1n03x5               g241(.a(new_n149), .b(new_n334), .c(new_n118), .out0(\s[8] ));
  xorb03aa1n02x5               g242(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


