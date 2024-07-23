// Benchmark "adder" written by ABC on Thu Jul 18 04:01:44 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n313, new_n314, new_n317;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  aoi112aa1n02x5               g002(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nor042aa1n04x5               g004(.a(\b[5] ), .b(\a[6] ), .o1(new_n100));
  oab012aa1n03x5               g005(.a(new_n100), .b(\a[5] ), .c(\b[4] ), .out0(new_n101));
  xorc02aa1n03x5               g006(.a(\a[8] ), .b(\b[7] ), .out0(new_n102));
  nor042aa1n03x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nand02aa1n06x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  norb02aa1n03x4               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nand42aa1d28x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  inv000aa1n02x5               g011(.a(new_n106), .o1(new_n107));
  nona23aa1n02x4               g012(.a(new_n102), .b(new_n105), .c(new_n101), .d(new_n107), .out0(new_n108));
  nona22aa1n12x5               g013(.a(new_n108), .b(new_n99), .c(new_n98), .out0(new_n109));
  nor022aa1n08x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  nanp02aa1n02x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  nand22aa1n03x5               g020(.a(\b[0] ), .b(\a[1] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  tech160nm_fioai012aa1n04x5   g022(.a(new_n115), .b(new_n117), .c(new_n116), .o1(new_n118));
  aoi012aa1n02x5               g023(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n119));
  oai012aa1n06x5               g024(.a(new_n119), .b(new_n114), .c(new_n118), .o1(new_n120));
  xnrc02aa1n02x5               g025(.a(\b[7] ), .b(\a[8] ), .out0(new_n121));
  norb02aa1n02x5               g026(.a(new_n106), .b(new_n100), .out0(new_n122));
  xorc02aa1n02x5               g027(.a(\a[5] ), .b(\b[4] ), .out0(new_n123));
  nano32aa1n06x5               g028(.a(new_n121), .b(new_n123), .c(new_n105), .d(new_n122), .out0(new_n124));
  xorc02aa1n06x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n109), .c(new_n124), .d(new_n120), .o1(new_n126));
  xorc02aa1n12x5               g031(.a(\a[10] ), .b(\b[9] ), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n126), .c(new_n97), .out0(\s[10] ));
  inv000aa1d42x5               g033(.a(new_n127), .o1(new_n129));
  oaih22aa1d12x5               g034(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n130));
  aob012aa1n12x5               g035(.a(new_n130), .b(\b[9] ), .c(\a[10] ), .out0(new_n131));
  aoai13aa1n04x5               g036(.a(new_n131), .b(new_n129), .c(new_n126), .d(new_n97), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n16x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n10x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nor022aa1n16x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nand42aa1n06x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  aoi112aa1n02x5               g043(.a(new_n138), .b(new_n134), .c(new_n132), .d(new_n135), .o1(new_n139));
  aoai13aa1n02x5               g044(.a(new_n138), .b(new_n134), .c(new_n132), .d(new_n135), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(\s[12] ));
  oai022aa1n02x5               g046(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n142));
  nanb02aa1n02x5               g047(.a(new_n103), .b(new_n104), .out0(new_n143));
  nano23aa1n02x4               g048(.a(new_n121), .b(new_n143), .c(new_n106), .d(new_n142), .out0(new_n144));
  nor043aa1n02x5               g049(.a(new_n144), .b(new_n99), .c(new_n98), .o1(new_n145));
  nanp02aa1n03x5               g050(.a(new_n124), .b(new_n120), .o1(new_n146));
  nano23aa1n06x5               g051(.a(new_n134), .b(new_n136), .c(new_n137), .d(new_n135), .out0(new_n147));
  nand23aa1d12x5               g052(.a(new_n147), .b(new_n125), .c(new_n127), .o1(new_n148));
  nona23aa1n09x5               g053(.a(new_n137), .b(new_n135), .c(new_n134), .d(new_n136), .out0(new_n149));
  oaih12aa1n02x5               g054(.a(new_n137), .b(new_n136), .c(new_n134), .o1(new_n150));
  oai012aa1n12x5               g055(.a(new_n150), .b(new_n149), .c(new_n131), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n148), .c(new_n146), .d(new_n145), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  tech160nm_finor002aa1n05x5   g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv000aa1d42x5               g063(.a(new_n148), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n109), .c(new_n124), .d(new_n120), .o1(new_n160));
  nor002aa1n03x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nona23aa1n02x5               g067(.a(new_n162), .b(new_n156), .c(new_n155), .d(new_n161), .out0(new_n163));
  oaih12aa1n02x5               g068(.a(new_n162), .b(new_n161), .c(new_n155), .o1(new_n164));
  aoai13aa1n03x5               g069(.a(new_n164), .b(new_n163), .c(new_n160), .d(new_n152), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  xnrc02aa1n12x5               g072(.a(\b[14] ), .b(\a[15] ), .out0(new_n168));
  inv000aa1n02x5               g073(.a(new_n168), .o1(new_n169));
  xnrc02aa1n12x5               g074(.a(\b[15] ), .b(\a[16] ), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(new_n171), .b(new_n167), .c(new_n165), .d(new_n169), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n171), .b(new_n167), .c(new_n165), .d(new_n169), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  nano23aa1n02x5               g079(.a(new_n155), .b(new_n161), .c(new_n162), .d(new_n156), .out0(new_n175));
  nano32aa1n03x7               g080(.a(new_n148), .b(new_n171), .c(new_n169), .d(new_n175), .out0(new_n176));
  aoai13aa1n09x5               g081(.a(new_n176), .b(new_n109), .c(new_n124), .d(new_n120), .o1(new_n177));
  nor043aa1n02x5               g082(.a(new_n163), .b(new_n170), .c(new_n168), .o1(new_n178));
  aoi112aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n179));
  nor042aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n180), .o1(new_n181));
  oai013aa1n03x4               g086(.a(new_n181), .b(new_n168), .c(new_n170), .d(new_n164), .o1(new_n182));
  aoi112aa1n06x5               g087(.a(new_n182), .b(new_n179), .c(new_n151), .d(new_n178), .o1(new_n183));
  nanp02aa1n09x5               g088(.a(new_n177), .b(new_n183), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g090(.a(\a[18] ), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\a[17] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\b[16] ), .o1(new_n188));
  oaoi03aa1n03x5               g093(.a(new_n187), .b(new_n188), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n186), .out0(\s[18] ));
  xroi22aa1d04x5               g095(.a(new_n187), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n191));
  nanp02aa1n02x5               g096(.a(new_n188), .b(new_n187), .o1(new_n192));
  oaoi03aa1n02x5               g097(.a(\a[18] ), .b(\b[17] ), .c(new_n192), .o1(new_n193));
  nor042aa1n04x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nand42aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n193), .c(new_n184), .d(new_n191), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(new_n196), .b(new_n193), .c(new_n184), .d(new_n191), .o1(new_n198));
  norb02aa1n02x7               g103(.a(new_n197), .b(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nand22aa1n03x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  nona22aa1n02x5               g108(.a(new_n197), .b(new_n203), .c(new_n194), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n203), .o1(new_n205));
  oaoi13aa1n06x5               g110(.a(new_n205), .b(new_n197), .c(\a[19] ), .d(\b[18] ), .o1(new_n206));
  norb02aa1n03x4               g111(.a(new_n204), .b(new_n206), .out0(\s[20] ));
  nano23aa1n06x5               g112(.a(new_n194), .b(new_n201), .c(new_n202), .d(new_n195), .out0(new_n208));
  nanp02aa1n02x5               g113(.a(new_n191), .b(new_n208), .o1(new_n209));
  oai022aa1n02x5               g114(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n210));
  oaib12aa1n06x5               g115(.a(new_n210), .b(new_n186), .c(\b[17] ), .out0(new_n211));
  nona23aa1n09x5               g116(.a(new_n202), .b(new_n195), .c(new_n194), .d(new_n201), .out0(new_n212));
  aoi012aa1n06x5               g117(.a(new_n201), .b(new_n194), .c(new_n202), .o1(new_n213));
  oai012aa1n18x5               g118(.a(new_n213), .b(new_n212), .c(new_n211), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n209), .c(new_n177), .d(new_n183), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  xorc02aa1n02x5               g124(.a(\a[22] ), .b(\b[21] ), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n218), .b(new_n220), .c(new_n216), .d(new_n219), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n220), .b(new_n218), .c(new_n216), .d(new_n219), .o1(new_n222));
  norb02aa1n02x7               g127(.a(new_n222), .b(new_n221), .out0(\s[22] ));
  inv000aa1d42x5               g128(.a(\a[21] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\a[22] ), .o1(new_n225));
  xroi22aa1d04x5               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  inv000aa1d42x5               g131(.a(\b[21] ), .o1(new_n227));
  oao003aa1n02x5               g132(.a(new_n225), .b(new_n227), .c(new_n218), .carry(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n214), .c(new_n226), .o1(new_n229));
  nanp03aa1n02x5               g134(.a(new_n226), .b(new_n191), .c(new_n208), .o1(new_n230));
  aoai13aa1n04x5               g135(.a(new_n229), .b(new_n230), .c(new_n177), .d(new_n183), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n03x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aoi112aa1n02x7               g140(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n237));
  norb02aa1n02x7               g142(.a(new_n237), .b(new_n236), .out0(\s[24] ));
  and002aa1n06x5               g143(.a(new_n235), .b(new_n234), .o(new_n239));
  inv000aa1n02x5               g144(.a(new_n239), .o1(new_n240));
  nano32aa1n02x4               g145(.a(new_n240), .b(new_n226), .c(new_n191), .d(new_n208), .out0(new_n241));
  inv040aa1n02x5               g146(.a(new_n213), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n226), .b(new_n242), .c(new_n208), .d(new_n193), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n228), .o1(new_n244));
  aoi112aa1n02x5               g149(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n245));
  oab012aa1n02x4               g150(.a(new_n245), .b(\a[24] ), .c(\b[23] ), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n240), .c(new_n243), .d(new_n244), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[25] ), .b(\b[24] ), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n247), .c(new_n184), .d(new_n241), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n248), .b(new_n247), .c(new_n184), .d(new_n241), .o1(new_n250));
  norb02aa1n02x7               g155(.a(new_n249), .b(new_n250), .out0(\s[25] ));
  nor042aa1n03x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  xorc02aa1n02x5               g157(.a(\a[26] ), .b(\b[25] ), .out0(new_n253));
  nona22aa1n02x5               g158(.a(new_n249), .b(new_n253), .c(new_n252), .out0(new_n254));
  inv000aa1d42x5               g159(.a(new_n252), .o1(new_n255));
  aobi12aa1n06x5               g160(.a(new_n253), .b(new_n249), .c(new_n255), .out0(new_n256));
  norb02aa1n03x4               g161(.a(new_n254), .b(new_n256), .out0(\s[26] ));
  nanp02aa1n03x5               g162(.a(new_n146), .b(new_n145), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n151), .b(new_n178), .o1(new_n259));
  nona22aa1n02x4               g164(.a(new_n259), .b(new_n182), .c(new_n179), .out0(new_n260));
  inv000aa1d42x5               g165(.a(\a[25] ), .o1(new_n261));
  inv040aa1d32x5               g166(.a(\a[26] ), .o1(new_n262));
  xroi22aa1d06x4               g167(.a(new_n261), .b(\b[24] ), .c(new_n262), .d(\b[25] ), .out0(new_n263));
  nano22aa1n03x7               g168(.a(new_n230), .b(new_n239), .c(new_n263), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n260), .c(new_n258), .d(new_n176), .o1(new_n265));
  oao003aa1n03x5               g170(.a(\a[26] ), .b(\b[25] ), .c(new_n255), .carry(new_n266));
  aobi12aa1n06x5               g171(.a(new_n266), .b(new_n247), .c(new_n263), .out0(new_n267));
  xorc02aa1n02x5               g172(.a(\a[27] ), .b(\b[26] ), .out0(new_n268));
  xnbna2aa1n03x5               g173(.a(new_n268), .b(new_n267), .c(new_n265), .out0(\s[27] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  inv040aa1n03x5               g175(.a(new_n270), .o1(new_n271));
  aobi12aa1n02x7               g176(.a(new_n268), .b(new_n267), .c(new_n265), .out0(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  nano22aa1n02x4               g178(.a(new_n272), .b(new_n271), .c(new_n273), .out0(new_n274));
  inv000aa1n02x5               g179(.a(new_n264), .o1(new_n275));
  tech160nm_fiaoi012aa1n05x5   g180(.a(new_n275), .b(new_n177), .c(new_n183), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n239), .b(new_n228), .c(new_n214), .d(new_n226), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n263), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n266), .b(new_n278), .c(new_n277), .d(new_n246), .o1(new_n279));
  oaih12aa1n02x5               g184(.a(new_n268), .b(new_n279), .c(new_n276), .o1(new_n280));
  tech160nm_fiaoi012aa1n02p5x5 g185(.a(new_n273), .b(new_n280), .c(new_n271), .o1(new_n281));
  nor002aa1n02x5               g186(.a(new_n281), .b(new_n274), .o1(\s[28] ));
  norb02aa1n02x5               g187(.a(new_n268), .b(new_n273), .out0(new_n283));
  oaih12aa1n02x5               g188(.a(new_n283), .b(new_n279), .c(new_n276), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n271), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  tech160nm_fiaoi012aa1n02p5x5 g191(.a(new_n286), .b(new_n284), .c(new_n285), .o1(new_n287));
  aobi12aa1n02x7               g192(.a(new_n283), .b(new_n267), .c(new_n265), .out0(new_n288));
  nano22aa1n03x5               g193(.a(new_n288), .b(new_n285), .c(new_n286), .out0(new_n289));
  nor002aa1n02x5               g194(.a(new_n287), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g196(.a(new_n268), .b(new_n286), .c(new_n273), .out0(new_n292));
  oaih12aa1n02x5               g197(.a(new_n292), .b(new_n279), .c(new_n276), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[29] ), .b(\a[30] ), .out0(new_n295));
  tech160nm_fiaoi012aa1n02p5x5 g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n02x7               g201(.a(new_n292), .b(new_n267), .c(new_n265), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  nor002aa1n02x5               g203(.a(new_n296), .b(new_n298), .o1(\s[30] ));
  norb02aa1n02x5               g204(.a(new_n292), .b(new_n295), .out0(new_n300));
  aobi12aa1n02x7               g205(.a(new_n300), .b(new_n267), .c(new_n265), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nano22aa1n03x5               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  oaih12aa1n02x5               g209(.a(new_n300), .b(new_n279), .c(new_n276), .o1(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n03x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xnrb03aa1n02x5               g212(.a(new_n118), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n118), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n120), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g216(.a(new_n120), .b(new_n123), .o1(new_n312));
  oaoi13aa1n02x5               g217(.a(new_n122), .b(new_n312), .c(\a[5] ), .d(\b[4] ), .o1(new_n313));
  oai112aa1n02x5               g218(.a(new_n312), .b(new_n122), .c(\b[4] ), .d(\a[5] ), .o1(new_n314));
  nanb02aa1n02x5               g219(.a(new_n313), .b(new_n314), .out0(\s[6] ));
  xnbna2aa1n03x5               g220(.a(new_n143), .b(new_n314), .c(new_n106), .out0(\s[7] ));
  aoi013aa1n02x4               g221(.a(new_n103), .b(new_n314), .c(new_n106), .d(new_n104), .o1(new_n317));
  xnrc02aa1n02x5               g222(.a(new_n317), .b(new_n102), .out0(\s[8] ));
  xnbna2aa1n03x5               g223(.a(new_n125), .b(new_n146), .c(new_n145), .out0(\s[9] ));
endmodule


