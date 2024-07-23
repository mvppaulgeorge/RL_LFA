// Benchmark "adder" written by ABC on Thu Jul 18 14:54:32 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n318,
    new_n319, new_n321, new_n322, new_n323, new_n325, new_n326, new_n327;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  nanb02aa1n06x5               g002(.a(\b[8] ), .b(new_n97), .out0(new_n98));
  inv000aa1d42x5               g003(.a(\b[2] ), .o1(new_n99));
  nanb02aa1n12x5               g004(.a(\a[3] ), .b(new_n99), .out0(new_n100));
  oaoi03aa1n09x5               g005(.a(\a[4] ), .b(\b[3] ), .c(new_n100), .o1(new_n101));
  nand42aa1n03x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nanp02aa1n12x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor042aa1n06x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  oai012aa1n12x5               g009(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nand42aa1n02x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nanb02aa1n03x5               g012(.a(new_n106), .b(new_n107), .out0(new_n108));
  nand42aa1n02x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nand02aa1n02x5               g014(.a(new_n100), .b(new_n109), .o1(new_n110));
  nor003aa1n06x5               g015(.a(new_n105), .b(new_n108), .c(new_n110), .o1(new_n111));
  nor002aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nanp02aa1n12x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n09x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n03x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .out0(new_n118));
  nor003aa1n04x5               g023(.a(new_n116), .b(new_n117), .c(new_n118), .o1(new_n119));
  oai012aa1n12x5               g024(.a(new_n119), .b(new_n111), .c(new_n101), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  oai022aa1n02x7               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  nano22aa1n03x7               g027(.a(new_n114), .b(new_n113), .c(new_n115), .out0(new_n123));
  oai022aa1n02x5               g028(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n124));
  aoai13aa1n12x5               g029(.a(new_n121), .b(new_n124), .c(new_n123), .d(new_n122), .o1(new_n125));
  nanp02aa1n06x5               g030(.a(new_n120), .b(new_n125), .o1(new_n126));
  xnrc02aa1n12x5               g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(new_n126), .b(new_n128), .o1(new_n129));
  tech160nm_fixorc02aa1n03p5x5 g034(.a(\a[10] ), .b(\b[9] ), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n129), .c(new_n98), .out0(\s[10] ));
  and002aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o(new_n132));
  oa0022aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .c(\b[8] ), .d(\a[9] ), .o(new_n133));
  aoai13aa1n06x5               g038(.a(new_n133), .b(new_n127), .c(new_n120), .d(new_n125), .o1(new_n134));
  nor002aa1d32x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1n08x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nona23aa1n03x5               g041(.a(new_n134), .b(new_n136), .c(new_n135), .d(new_n132), .out0(new_n137));
  inv000aa1d42x5               g042(.a(new_n135), .o1(new_n138));
  aboi22aa1n03x5               g043(.a(new_n132), .b(new_n134), .c(new_n138), .d(new_n136), .out0(new_n139));
  norb02aa1n02x5               g044(.a(new_n137), .b(new_n139), .out0(\s[11] ));
  nor022aa1n08x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1n06x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n137), .c(new_n138), .out0(\s[12] ));
  nona23aa1n09x5               g049(.a(new_n142), .b(new_n136), .c(new_n135), .d(new_n141), .out0(new_n145));
  norb03aa1n12x5               g050(.a(new_n130), .b(new_n145), .c(new_n127), .out0(new_n146));
  aoi012aa1n06x5               g051(.a(new_n141), .b(new_n135), .c(new_n142), .o1(new_n147));
  oaoi03aa1n09x5               g052(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n148));
  oaib12aa1n02x5               g053(.a(new_n147), .b(new_n145), .c(new_n148), .out0(new_n149));
  nor042aa1n09x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nand42aa1n03x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  norb02aa1n02x5               g056(.a(new_n151), .b(new_n150), .out0(new_n152));
  aoai13aa1n03x5               g057(.a(new_n152), .b(new_n149), .c(new_n126), .d(new_n146), .o1(new_n153));
  aoi112aa1n02x5               g058(.a(new_n152), .b(new_n149), .c(new_n126), .d(new_n146), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(\s[13] ));
  inv000aa1d42x5               g060(.a(new_n150), .o1(new_n156));
  xorc02aa1n02x5               g061(.a(\a[14] ), .b(\b[13] ), .out0(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n153), .c(new_n156), .out0(\s[14] ));
  xnrc02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .out0(new_n159));
  nano22aa1n03x7               g064(.a(new_n159), .b(new_n156), .c(new_n151), .out0(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n149), .c(new_n126), .d(new_n146), .o1(new_n161));
  oao003aa1n02x5               g066(.a(\a[14] ), .b(\b[13] ), .c(new_n156), .carry(new_n162));
  xorc02aa1n12x5               g067(.a(\a[15] ), .b(\b[14] ), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n161), .c(new_n162), .out0(\s[15] ));
  aob012aa1n03x5               g069(.a(new_n163), .b(new_n161), .c(new_n162), .out0(new_n165));
  nor042aa1n06x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n163), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n167), .b(new_n168), .c(new_n161), .d(new_n162), .o1(new_n169));
  tech160nm_fixorc02aa1n02p5x5 g074(.a(\a[16] ), .b(\b[15] ), .out0(new_n170));
  norp02aa1n02x5               g075(.a(new_n170), .b(new_n166), .o1(new_n171));
  aoi022aa1n03x5               g076(.a(new_n169), .b(new_n170), .c(new_n165), .d(new_n171), .o1(\s[16] ));
  nanp02aa1n02x5               g077(.a(new_n157), .b(new_n152), .o1(new_n173));
  nand02aa1n04x5               g078(.a(new_n170), .b(new_n163), .o1(new_n174));
  nona22aa1n09x5               g079(.a(new_n146), .b(new_n173), .c(new_n174), .out0(new_n175));
  inv000aa1n04x5               g080(.a(new_n175), .o1(new_n176));
  nand42aa1n06x5               g081(.a(new_n176), .b(new_n126), .o1(new_n177));
  nano23aa1n03x7               g082(.a(new_n135), .b(new_n141), .c(new_n142), .d(new_n136), .out0(new_n178));
  inv020aa1n03x5               g083(.a(new_n147), .o1(new_n179));
  aoai13aa1n04x5               g084(.a(new_n160), .b(new_n179), .c(new_n178), .d(new_n148), .o1(new_n180));
  aoi012aa1n02x5               g085(.a(new_n174), .b(new_n180), .c(new_n162), .o1(new_n181));
  oao003aa1n12x5               g086(.a(\a[16] ), .b(\b[15] ), .c(new_n167), .carry(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  nona22aa1n09x5               g088(.a(new_n177), .b(new_n181), .c(new_n183), .out0(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  nanb02aa1n12x5               g091(.a(\b[16] ), .b(new_n186), .out0(new_n187));
  aoi012aa1n12x5               g092(.a(new_n175), .b(new_n120), .c(new_n125), .o1(new_n188));
  aoai13aa1n06x5               g093(.a(new_n182), .b(new_n174), .c(new_n180), .d(new_n162), .o1(new_n189));
  xorc02aa1n02x5               g094(.a(\a[17] ), .b(\b[16] ), .out0(new_n190));
  oaih12aa1n02x5               g095(.a(new_n190), .b(new_n189), .c(new_n188), .o1(new_n191));
  tech160nm_fixorc02aa1n02p5x5 g096(.a(\a[18] ), .b(\b[17] ), .out0(new_n192));
  xnbna2aa1n03x5               g097(.a(new_n192), .b(new_n191), .c(new_n187), .out0(\s[18] ));
  inv000aa1d42x5               g098(.a(\a[18] ), .o1(new_n194));
  xroi22aa1d04x5               g099(.a(new_n186), .b(\b[16] ), .c(new_n194), .d(\b[17] ), .out0(new_n195));
  tech160nm_fioai012aa1n05x5   g100(.a(new_n195), .b(new_n189), .c(new_n188), .o1(new_n196));
  oai022aa1n02x7               g101(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n197));
  oaib12aa1n09x5               g102(.a(new_n197), .b(new_n194), .c(\b[17] ), .out0(new_n198));
  nor002aa1d32x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand22aa1n09x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n196), .c(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n02x5               g109(.a(\a[18] ), .b(\b[17] ), .c(new_n187), .o1(new_n205));
  aoai13aa1n03x5               g110(.a(new_n202), .b(new_n205), .c(new_n184), .d(new_n195), .o1(new_n206));
  inv000aa1n02x5               g111(.a(new_n199), .o1(new_n207));
  aoai13aa1n02x5               g112(.a(new_n207), .b(new_n201), .c(new_n196), .d(new_n198), .o1(new_n208));
  nor002aa1n04x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand02aa1d08x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  aoib12aa1n02x5               g116(.a(new_n199), .b(new_n210), .c(new_n209), .out0(new_n212));
  aoi022aa1n02x5               g117(.a(new_n208), .b(new_n211), .c(new_n206), .d(new_n212), .o1(\s[20] ));
  nano23aa1n06x5               g118(.a(new_n199), .b(new_n209), .c(new_n210), .d(new_n200), .out0(new_n214));
  nand23aa1n06x5               g119(.a(new_n214), .b(new_n190), .c(new_n192), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  oai012aa1n06x5               g121(.a(new_n216), .b(new_n189), .c(new_n188), .o1(new_n217));
  nona23aa1n09x5               g122(.a(new_n210), .b(new_n200), .c(new_n199), .d(new_n209), .out0(new_n218));
  oaoi03aa1n09x5               g123(.a(\a[20] ), .b(\b[19] ), .c(new_n207), .o1(new_n219));
  oabi12aa1n18x5               g124(.a(new_n219), .b(new_n218), .c(new_n198), .out0(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xnbna2aa1n03x5               g127(.a(new_n222), .b(new_n217), .c(new_n221), .out0(\s[21] ));
  aoai13aa1n06x5               g128(.a(new_n222), .b(new_n220), .c(new_n184), .d(new_n216), .o1(new_n224));
  nor002aa1d32x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n222), .o1(new_n227));
  aoai13aa1n02x5               g132(.a(new_n226), .b(new_n227), .c(new_n217), .d(new_n221), .o1(new_n228));
  xorc02aa1n02x5               g133(.a(\a[22] ), .b(\b[21] ), .out0(new_n229));
  norp02aa1n02x5               g134(.a(new_n229), .b(new_n225), .o1(new_n230));
  aoi022aa1n03x5               g135(.a(new_n228), .b(new_n229), .c(new_n224), .d(new_n230), .o1(\s[22] ));
  nanp02aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  xnrc02aa1n02x5               g137(.a(\b[21] ), .b(\a[22] ), .out0(new_n233));
  nano22aa1n03x7               g138(.a(new_n233), .b(new_n226), .c(new_n232), .out0(new_n234));
  norb02aa1n02x5               g139(.a(new_n234), .b(new_n215), .out0(new_n235));
  oai012aa1n06x5               g140(.a(new_n235), .b(new_n189), .c(new_n188), .o1(new_n236));
  oao003aa1n06x5               g141(.a(\a[22] ), .b(\b[21] ), .c(new_n226), .carry(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  aoi012aa1n02x5               g143(.a(new_n238), .b(new_n220), .c(new_n234), .o1(new_n239));
  nand42aa1n02x5               g144(.a(new_n236), .b(new_n239), .o1(new_n240));
  xorc02aa1n12x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  aoi112aa1n02x5               g146(.a(new_n241), .b(new_n238), .c(new_n220), .d(new_n234), .o1(new_n242));
  aoi022aa1n02x5               g147(.a(new_n240), .b(new_n241), .c(new_n236), .d(new_n242), .o1(\s[23] ));
  nanp02aa1n02x5               g148(.a(new_n240), .b(new_n241), .o1(new_n244));
  nor042aa1n06x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n241), .o1(new_n247));
  aoai13aa1n02x5               g152(.a(new_n246), .b(new_n247), .c(new_n236), .d(new_n239), .o1(new_n248));
  tech160nm_fixorc02aa1n04x5   g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  norp02aa1n02x5               g154(.a(new_n249), .b(new_n245), .o1(new_n250));
  aoi022aa1n02x7               g155(.a(new_n248), .b(new_n249), .c(new_n244), .d(new_n250), .o1(\s[24] ));
  nano32aa1n03x7               g156(.a(new_n215), .b(new_n249), .c(new_n234), .d(new_n241), .out0(new_n252));
  oai012aa1n06x5               g157(.a(new_n252), .b(new_n189), .c(new_n188), .o1(new_n253));
  aoai13aa1n06x5               g158(.a(new_n234), .b(new_n219), .c(new_n214), .d(new_n205), .o1(new_n254));
  and002aa1n12x5               g159(.a(new_n249), .b(new_n241), .o(new_n255));
  inv020aa1n04x5               g160(.a(new_n255), .o1(new_n256));
  oao003aa1n02x5               g161(.a(\a[24] ), .b(\b[23] ), .c(new_n246), .carry(new_n257));
  aoai13aa1n12x5               g162(.a(new_n257), .b(new_n256), .c(new_n254), .d(new_n237), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  xnbna2aa1n03x5               g165(.a(new_n260), .b(new_n253), .c(new_n259), .out0(\s[25] ));
  aoai13aa1n06x5               g166(.a(new_n260), .b(new_n258), .c(new_n184), .d(new_n252), .o1(new_n262));
  nor042aa1n03x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n260), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n264), .b(new_n265), .c(new_n253), .d(new_n259), .o1(new_n266));
  xorc02aa1n02x5               g171(.a(\a[26] ), .b(\b[25] ), .out0(new_n267));
  norp02aa1n02x5               g172(.a(new_n267), .b(new_n263), .o1(new_n268));
  aoi022aa1n02x5               g173(.a(new_n266), .b(new_n267), .c(new_n262), .d(new_n268), .o1(\s[26] ));
  and002aa1n12x5               g174(.a(new_n267), .b(new_n260), .o(new_n270));
  inv000aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  nano23aa1n06x5               g176(.a(new_n271), .b(new_n215), .c(new_n255), .d(new_n234), .out0(new_n272));
  oai012aa1n06x5               g177(.a(new_n272), .b(new_n189), .c(new_n188), .o1(new_n273));
  oao003aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .c(new_n264), .carry(new_n274));
  aobi12aa1n12x5               g179(.a(new_n274), .b(new_n258), .c(new_n270), .out0(new_n275));
  xorc02aa1n12x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnbna2aa1n03x5               g181(.a(new_n276), .b(new_n275), .c(new_n273), .out0(\s[27] ));
  aoai13aa1n03x5               g182(.a(new_n255), .b(new_n238), .c(new_n220), .d(new_n234), .o1(new_n278));
  aoai13aa1n04x5               g183(.a(new_n274), .b(new_n271), .c(new_n278), .d(new_n257), .o1(new_n279));
  aoai13aa1n03x5               g184(.a(new_n276), .b(new_n279), .c(new_n184), .d(new_n272), .o1(new_n280));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  inv000aa1n03x5               g186(.a(new_n281), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n276), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n282), .b(new_n283), .c(new_n275), .d(new_n273), .o1(new_n284));
  tech160nm_fixorc02aa1n03p5x5 g189(.a(\a[28] ), .b(\b[27] ), .out0(new_n285));
  norp02aa1n02x5               g190(.a(new_n285), .b(new_n281), .o1(new_n286));
  aoi022aa1n03x5               g191(.a(new_n284), .b(new_n285), .c(new_n280), .d(new_n286), .o1(\s[28] ));
  and002aa1n02x5               g192(.a(new_n285), .b(new_n276), .o(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n279), .c(new_n184), .d(new_n272), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n288), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n291));
  aoai13aa1n02x7               g196(.a(new_n291), .b(new_n290), .c(new_n275), .d(new_n273), .o1(new_n292));
  xorc02aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .out0(new_n293));
  norb02aa1n02x5               g198(.a(new_n291), .b(new_n293), .out0(new_n294));
  aoi022aa1n03x5               g199(.a(new_n292), .b(new_n293), .c(new_n289), .d(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n12x5               g201(.a(new_n283), .b(new_n285), .c(new_n293), .out0(new_n297));
  aoai13aa1n02x5               g202(.a(new_n297), .b(new_n279), .c(new_n184), .d(new_n272), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n297), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n299), .c(new_n275), .d(new_n273), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .out0(new_n302));
  norb02aa1n02x5               g207(.a(new_n300), .b(new_n302), .out0(new_n303));
  aoi022aa1n03x5               g208(.a(new_n301), .b(new_n302), .c(new_n298), .d(new_n303), .o1(\s[30] ));
  xorc02aa1n02x5               g209(.a(\a[31] ), .b(\b[30] ), .out0(new_n305));
  nano32aa1d12x5               g210(.a(new_n283), .b(new_n302), .c(new_n285), .d(new_n293), .out0(new_n306));
  aoai13aa1n02x5               g211(.a(new_n306), .b(new_n279), .c(new_n184), .d(new_n272), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n306), .o1(new_n308));
  oao003aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .c(new_n300), .carry(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n308), .c(new_n275), .d(new_n273), .o1(new_n310));
  and002aa1n02x5               g215(.a(\b[29] ), .b(\a[30] ), .o(new_n311));
  oabi12aa1n02x5               g216(.a(new_n305), .b(\a[30] ), .c(\b[29] ), .out0(new_n312));
  oab012aa1n02x4               g217(.a(new_n312), .b(new_n300), .c(new_n311), .out0(new_n313));
  aoi022aa1n03x5               g218(.a(new_n310), .b(new_n305), .c(new_n307), .d(new_n313), .o1(\s[31] ));
  xnbna2aa1n03x5               g219(.a(new_n105), .b(new_n109), .c(new_n100), .out0(\s[3] ));
  oaoi03aa1n02x5               g220(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  oabi12aa1n02x5               g222(.a(new_n117), .b(new_n111), .c(new_n101), .out0(new_n318));
  norb03aa1n02x5               g223(.a(new_n117), .b(new_n111), .c(new_n101), .out0(new_n319));
  norb02aa1n02x5               g224(.a(new_n318), .b(new_n319), .out0(\s[5] ));
  norb02aa1n02x5               g225(.a(new_n113), .b(new_n112), .out0(new_n321));
  orn002aa1n02x5               g226(.a(\a[5] ), .b(\b[4] ), .o(new_n322));
  nanb03aa1n02x5               g227(.a(new_n122), .b(new_n318), .c(new_n113), .out0(new_n323));
  aoai13aa1n02x5               g228(.a(new_n323), .b(new_n321), .c(new_n318), .d(new_n322), .o1(\s[6] ));
  inv000aa1d42x5               g229(.a(new_n114), .o1(new_n325));
  aoi022aa1n02x5               g230(.a(new_n323), .b(new_n113), .c(new_n325), .d(new_n115), .o1(new_n326));
  nanp02aa1n02x5               g231(.a(new_n323), .b(new_n123), .o1(new_n327));
  norb02aa1n02x5               g232(.a(new_n327), .b(new_n326), .out0(\s[7] ));
  xobna2aa1n03x5               g233(.a(new_n118), .b(new_n327), .c(new_n325), .out0(\s[8] ));
  xnbna2aa1n03x5               g234(.a(new_n128), .b(new_n120), .c(new_n125), .out0(\s[9] ));
endmodule

