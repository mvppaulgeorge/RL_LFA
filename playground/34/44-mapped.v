// Benchmark "adder" written by ABC on Thu Jul 18 05:47:26 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n321, new_n323, new_n324, new_n326,
    new_n327, new_n329, new_n330, new_n332, new_n333, new_n334, new_n336,
    new_n337;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand02aa1d28x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nor042aa1n09x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nand02aa1d24x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nor042aa1d18x5               g005(.a(\b[8] ), .b(\a[9] ), .o1(new_n101));
  nand02aa1d10x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nor022aa1n16x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  nor002aa1d32x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nanp02aa1n04x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nona23aa1d18x5               g010(.a(new_n102), .b(new_n105), .c(new_n104), .d(new_n103), .out0(new_n106));
  inv000aa1d42x5               g011(.a(\a[6] ), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\b[5] ), .o1(new_n108));
  nor042aa1n04x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  oaoi03aa1n12x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  norp02aa1n02x5               g015(.a(new_n106), .b(new_n110), .o1(new_n111));
  nor042aa1n04x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  nand22aa1n09x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  nand22aa1n12x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  aoi012aa1n12x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  nor002aa1d32x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nanp02aa1n06x5               g021(.a(\b[3] ), .b(\a[4] ), .o1(new_n117));
  nor002aa1d32x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nanp02aa1n04x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nona23aa1n09x5               g024(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n120));
  tech160nm_fioai012aa1n03p5x5 g025(.a(new_n117), .b(new_n118), .c(new_n116), .o1(new_n121));
  oai012aa1n12x5               g026(.a(new_n121), .b(new_n120), .c(new_n115), .o1(new_n122));
  tech160nm_fixnrc02aa1n05x5   g027(.a(\b[5] ), .b(\a[6] ), .out0(new_n123));
  xnrc02aa1n06x5               g028(.a(\b[4] ), .b(\a[5] ), .out0(new_n124));
  nor043aa1n02x5               g029(.a(new_n106), .b(new_n123), .c(new_n124), .o1(new_n125));
  nanp02aa1n03x5               g030(.a(new_n122), .b(new_n125), .o1(new_n126));
  oa0012aa1n03x5               g031(.a(new_n102), .b(new_n103), .c(new_n104), .o(new_n127));
  nona32aa1n02x4               g032(.a(new_n126), .b(new_n111), .c(new_n127), .d(new_n101), .out0(new_n128));
  xobna2aa1n03x5               g033(.a(new_n99), .b(new_n128), .c(new_n100), .out0(\s[10] ));
  nand23aa1n03x5               g034(.a(new_n128), .b(new_n99), .c(new_n100), .o1(new_n130));
  tech160nm_fiao0012aa1n02p5x5 g035(.a(new_n98), .b(new_n101), .c(new_n97), .o(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n130), .c(new_n132), .out0(\s[11] ));
  inv000aa1d42x5               g041(.a(new_n133), .o1(new_n137));
  aob012aa1n03x5               g042(.a(new_n135), .b(new_n130), .c(new_n132), .out0(new_n138));
  nor042aa1n09x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand42aa1d28x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  norb03aa1n02x5               g046(.a(new_n140), .b(new_n133), .c(new_n139), .out0(new_n142));
  nand42aa1n03x5               g047(.a(new_n138), .b(new_n142), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n141), .c(new_n138), .d(new_n137), .o1(\s[12] ));
  oabi12aa1n18x5               g049(.a(new_n127), .b(new_n106), .c(new_n110), .out0(new_n145));
  nano23aa1n09x5               g050(.a(new_n133), .b(new_n139), .c(new_n140), .d(new_n134), .out0(new_n146));
  nano23aa1n09x5               g051(.a(new_n101), .b(new_n98), .c(new_n97), .d(new_n100), .out0(new_n147));
  nand22aa1n09x5               g052(.a(new_n147), .b(new_n146), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  aoai13aa1n03x5               g054(.a(new_n149), .b(new_n145), .c(new_n122), .d(new_n125), .o1(new_n150));
  aoai13aa1n02x7               g055(.a(new_n134), .b(new_n98), .c(new_n101), .d(new_n97), .o1(new_n151));
  nona22aa1n02x4               g056(.a(new_n151), .b(new_n139), .c(new_n133), .out0(new_n152));
  and002aa1n06x5               g057(.a(new_n152), .b(new_n140), .o(new_n153));
  inv000aa1n02x5               g058(.a(new_n153), .o1(new_n154));
  nor022aa1n12x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand42aa1d28x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n150), .c(new_n154), .out0(\s[13] ));
  inv000aa1n02x5               g063(.a(new_n145), .o1(new_n159));
  nanp02aa1n06x5               g064(.a(new_n126), .b(new_n159), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n157), .b(new_n153), .c(new_n160), .d(new_n149), .o1(new_n161));
  nor002aa1d32x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  inv000aa1d42x5               g067(.a(\a[13] ), .o1(new_n163));
  inv000aa1d42x5               g068(.a(\b[12] ), .o1(new_n164));
  nand42aa1d28x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  aboi22aa1n03x5               g070(.a(new_n162), .b(new_n165), .c(new_n163), .d(new_n164), .out0(new_n166));
  nano23aa1d15x5               g071(.a(new_n155), .b(new_n162), .c(new_n165), .d(new_n156), .out0(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n165), .b(new_n162), .c(new_n163), .d(new_n164), .o1(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n168), .c(new_n150), .d(new_n154), .o1(new_n170));
  aboi22aa1n03x5               g075(.a(new_n162), .b(new_n170), .c(new_n161), .d(new_n166), .out0(\s[14] ));
  aoai13aa1n06x5               g076(.a(new_n167), .b(new_n153), .c(new_n160), .d(new_n149), .o1(new_n172));
  nor002aa1n12x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nand02aa1d20x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nanb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  xobna2aa1n03x5               g080(.a(new_n175), .b(new_n172), .c(new_n169), .out0(\s[15] ));
  nor002aa1d32x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand02aa1d28x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n177), .b(new_n178), .out0(new_n179));
  aoai13aa1n04x5               g084(.a(new_n179), .b(new_n173), .c(new_n170), .d(new_n174), .o1(new_n180));
  oai022aa1n02x5               g085(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n178), .b(new_n175), .c(new_n172), .d(new_n169), .o1(new_n182));
  oai012aa1n03x5               g087(.a(new_n180), .b(new_n182), .c(new_n181), .o1(\s[16] ));
  nano23aa1n02x4               g088(.a(new_n173), .b(new_n177), .c(new_n178), .d(new_n174), .out0(new_n184));
  nano22aa1n03x7               g089(.a(new_n148), .b(new_n167), .c(new_n184), .out0(new_n185));
  aoai13aa1n06x5               g090(.a(new_n185), .b(new_n145), .c(new_n122), .d(new_n125), .o1(new_n186));
  nano22aa1n03x5               g091(.a(new_n177), .b(new_n174), .c(new_n178), .out0(new_n187));
  oai012aa1n02x5               g092(.a(new_n140), .b(\b[12] ), .c(\a[13] ), .o1(new_n188));
  nona23aa1n03x5               g093(.a(new_n165), .b(new_n156), .c(new_n173), .d(new_n162), .out0(new_n189));
  nor002aa1n02x5               g094(.a(new_n189), .b(new_n188), .o1(new_n190));
  oai122aa1n02x7               g095(.a(new_n165), .b(new_n162), .c(new_n155), .d(\b[14] ), .e(\a[15] ), .o1(new_n191));
  oai012aa1n02x5               g096(.a(new_n178), .b(new_n177), .c(new_n173), .o1(new_n192));
  oaib12aa1n03x5               g097(.a(new_n192), .b(new_n191), .c(new_n187), .out0(new_n193));
  aoi013aa1n06x5               g098(.a(new_n193), .b(new_n190), .c(new_n152), .d(new_n187), .o1(new_n194));
  nanp02aa1n09x5               g099(.a(new_n186), .b(new_n194), .o1(new_n195));
  nor042aa1d18x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  nand42aa1d28x5               g101(.a(\b[16] ), .b(\a[17] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  aboi22aa1n03x5               g103(.a(new_n196), .b(new_n197), .c(new_n181), .d(new_n178), .out0(new_n199));
  nona22aa1n02x4               g104(.a(new_n170), .b(new_n175), .c(new_n179), .out0(new_n200));
  aoi022aa1n02x5               g105(.a(new_n200), .b(new_n199), .c(new_n195), .d(new_n198), .o1(\s[17] ));
  nor042aa1d18x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nand42aa1d28x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  obai22aa1n02x7               g108(.a(new_n203), .b(new_n202), .c(\a[17] ), .d(\b[16] ), .out0(new_n204));
  aoi012aa1n02x5               g109(.a(new_n204), .b(new_n195), .c(new_n198), .o1(new_n205));
  nano23aa1d15x5               g110(.a(new_n196), .b(new_n202), .c(new_n203), .d(new_n197), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  oai012aa1n02x5               g112(.a(new_n203), .b(new_n202), .c(new_n196), .o1(new_n208));
  aoai13aa1n04x5               g113(.a(new_n208), .b(new_n207), .c(new_n186), .d(new_n194), .o1(new_n209));
  aoib12aa1n02x5               g114(.a(new_n205), .b(new_n209), .c(new_n202), .out0(\s[18] ));
  nanp02aa1n03x5               g115(.a(new_n195), .b(new_n206), .o1(new_n211));
  xorc02aa1n12x5               g116(.a(\a[19] ), .b(\b[18] ), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n211), .c(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1d32x5               g119(.a(\a[19] ), .o1(new_n215));
  inv000aa1d42x5               g120(.a(\b[18] ), .o1(new_n216));
  oaoi03aa1n03x5               g121(.a(new_n215), .b(new_n216), .c(new_n209), .o1(new_n217));
  orn002aa1n24x5               g122(.a(\a[20] ), .b(\b[19] ), .o(new_n218));
  nand42aa1n16x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nanp02aa1n02x5               g124(.a(new_n209), .b(new_n212), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n219), .o1(new_n221));
  oai022aa1n02x5               g126(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n222));
  nona22aa1n02x5               g127(.a(new_n220), .b(new_n221), .c(new_n222), .out0(new_n223));
  aoai13aa1n02x5               g128(.a(new_n223), .b(new_n217), .c(new_n219), .d(new_n218), .o1(\s[20] ));
  xorc02aa1n12x5               g129(.a(\a[20] ), .b(\b[19] ), .out0(new_n225));
  nand23aa1d12x5               g130(.a(new_n206), .b(new_n212), .c(new_n225), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  oai112aa1n06x5               g132(.a(new_n218), .b(new_n219), .c(new_n216), .d(new_n215), .o1(new_n228));
  nand42aa1n02x5               g133(.a(new_n216), .b(new_n215), .o1(new_n229));
  oai112aa1n03x5               g134(.a(new_n203), .b(new_n229), .c(new_n202), .d(new_n196), .o1(new_n230));
  nanp02aa1n02x5               g135(.a(new_n222), .b(new_n219), .o1(new_n231));
  oai012aa1n06x5               g136(.a(new_n231), .b(new_n230), .c(new_n228), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[21] ), .b(\b[20] ), .out0(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n232), .c(new_n195), .d(new_n227), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(new_n233), .b(new_n232), .c(new_n195), .d(new_n227), .o1(new_n235));
  norb02aa1n03x4               g140(.a(new_n234), .b(new_n235), .out0(\s[21] ));
  inv000aa1d42x5               g141(.a(\a[21] ), .o1(new_n237));
  nanb02aa1n02x5               g142(.a(\b[20] ), .b(new_n237), .out0(new_n238));
  xorc02aa1n02x5               g143(.a(\a[22] ), .b(\b[21] ), .out0(new_n239));
  and002aa1n02x5               g144(.a(\b[21] ), .b(\a[22] ), .o(new_n240));
  oai022aa1n02x5               g145(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n241));
  nona22aa1n02x5               g146(.a(new_n234), .b(new_n240), .c(new_n241), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n239), .c(new_n238), .d(new_n234), .o1(\s[22] ));
  inv000aa1d42x5               g148(.a(\a[22] ), .o1(new_n244));
  xroi22aa1d06x4               g149(.a(new_n237), .b(\b[20] ), .c(new_n244), .d(\b[21] ), .out0(new_n245));
  norb02aa1n02x5               g150(.a(new_n245), .b(new_n226), .out0(new_n246));
  oaoi03aa1n02x5               g151(.a(\a[22] ), .b(\b[21] ), .c(new_n238), .o1(new_n247));
  tech160nm_fiao0012aa1n02p5x5 g152(.a(new_n247), .b(new_n232), .c(new_n245), .o(new_n248));
  xorc02aa1n12x5               g153(.a(\a[23] ), .b(\b[22] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n248), .c(new_n195), .d(new_n246), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(new_n249), .b(new_n248), .c(new_n195), .d(new_n246), .o1(new_n251));
  norb02aa1n03x4               g156(.a(new_n250), .b(new_n251), .out0(\s[23] ));
  norp02aa1n02x5               g157(.a(\b[22] ), .b(\a[23] ), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  tech160nm_fixorc02aa1n02p5x5 g159(.a(\a[24] ), .b(\b[23] ), .out0(new_n255));
  and002aa1n02x5               g160(.a(\b[23] ), .b(\a[24] ), .o(new_n256));
  oai022aa1n02x5               g161(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n257));
  nona22aa1n03x5               g162(.a(new_n250), .b(new_n256), .c(new_n257), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n258), .b(new_n255), .c(new_n254), .d(new_n250), .o1(\s[24] ));
  nano32aa1n02x5               g164(.a(new_n226), .b(new_n255), .c(new_n245), .d(new_n249), .out0(new_n260));
  and002aa1n02x5               g165(.a(new_n255), .b(new_n249), .o(new_n261));
  aoai13aa1n04x5               g166(.a(new_n261), .b(new_n247), .c(new_n232), .d(new_n245), .o1(new_n262));
  aob012aa1n02x5               g167(.a(new_n257), .b(\b[23] ), .c(\a[24] ), .out0(new_n263));
  nanp02aa1n02x5               g168(.a(new_n262), .b(new_n263), .o1(new_n264));
  xnrc02aa1n12x5               g169(.a(\b[24] ), .b(\a[25] ), .out0(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n264), .c(new_n195), .d(new_n260), .o1(new_n267));
  aoi112aa1n02x5               g172(.a(new_n266), .b(new_n264), .c(new_n195), .d(new_n260), .o1(new_n268));
  norb02aa1n03x4               g173(.a(new_n267), .b(new_n268), .out0(\s[25] ));
  norp02aa1n02x5               g174(.a(\b[24] ), .b(\a[25] ), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .out0(new_n272));
  and002aa1n02x5               g177(.a(\b[25] ), .b(\a[26] ), .o(new_n273));
  oai022aa1n02x5               g178(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n274));
  nona22aa1n03x5               g179(.a(new_n267), .b(new_n273), .c(new_n274), .out0(new_n275));
  aoai13aa1n03x5               g180(.a(new_n275), .b(new_n272), .c(new_n271), .d(new_n267), .o1(\s[26] ));
  nanb02aa1n02x5               g181(.a(new_n265), .b(new_n272), .out0(new_n277));
  nona23aa1n09x5               g182(.a(new_n245), .b(new_n261), .c(new_n226), .d(new_n277), .out0(new_n278));
  aoi012aa1n12x5               g183(.a(new_n278), .b(new_n186), .c(new_n194), .o1(new_n279));
  aob012aa1n02x5               g184(.a(new_n274), .b(\b[25] ), .c(\a[26] ), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n277), .c(new_n262), .d(new_n263), .o1(new_n281));
  xorc02aa1n02x5               g186(.a(\a[27] ), .b(\b[26] ), .out0(new_n282));
  tech160nm_fioai012aa1n04x5   g187(.a(new_n282), .b(new_n281), .c(new_n279), .o1(new_n283));
  norp03aa1n02x5               g188(.a(new_n281), .b(new_n279), .c(new_n282), .o1(new_n284));
  norb02aa1n03x4               g189(.a(new_n283), .b(new_n284), .out0(\s[27] ));
  norp02aa1n02x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n286), .o1(new_n287));
  xorc02aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .out0(new_n288));
  oai022aa1d24x5               g193(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n289));
  aoi012aa1n02x5               g194(.a(new_n289), .b(\a[28] ), .c(\b[27] ), .o1(new_n290));
  tech160nm_finand02aa1n03p5x5 g195(.a(new_n283), .b(new_n290), .o1(new_n291));
  aoai13aa1n03x5               g196(.a(new_n291), .b(new_n288), .c(new_n283), .d(new_n287), .o1(\s[28] ));
  xorc02aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .out0(new_n293));
  and002aa1n02x5               g198(.a(new_n288), .b(new_n282), .o(new_n294));
  oaih12aa1n02x5               g199(.a(new_n294), .b(new_n281), .c(new_n279), .o1(new_n295));
  inv000aa1d42x5               g200(.a(\b[27] ), .o1(new_n296));
  oaib12aa1n09x5               g201(.a(new_n289), .b(new_n296), .c(\a[28] ), .out0(new_n297));
  nand43aa1n03x5               g202(.a(new_n295), .b(new_n297), .c(new_n293), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n297), .o1(new_n299));
  oaoi13aa1n02x5               g204(.a(new_n299), .b(new_n294), .c(new_n281), .d(new_n279), .o1(new_n300));
  oai012aa1n03x5               g205(.a(new_n298), .b(new_n300), .c(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g207(.a(new_n282), .b(new_n293), .c(new_n288), .o(new_n303));
  tech160nm_fioaoi03aa1n03p5x5 g208(.a(\a[29] ), .b(\b[28] ), .c(new_n297), .o1(new_n304));
  oaoi13aa1n03x5               g209(.a(new_n304), .b(new_n303), .c(new_n281), .d(new_n279), .o1(new_n305));
  xorc02aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .out0(new_n306));
  oaih12aa1n02x5               g211(.a(new_n303), .b(new_n281), .c(new_n279), .o1(new_n307));
  norb02aa1n03x5               g212(.a(new_n306), .b(new_n304), .out0(new_n308));
  tech160nm_finand02aa1n03p5x5 g213(.a(new_n307), .b(new_n308), .o1(new_n309));
  oaih12aa1n02x5               g214(.a(new_n309), .b(new_n305), .c(new_n306), .o1(\s[30] ));
  and003aa1n02x5               g215(.a(new_n294), .b(new_n306), .c(new_n293), .o(new_n311));
  aoi012aa1n02x5               g216(.a(new_n308), .b(\a[30] ), .c(\b[29] ), .o1(new_n312));
  oaoi13aa1n03x5               g217(.a(new_n312), .b(new_n311), .c(new_n281), .d(new_n279), .o1(new_n313));
  xorc02aa1n02x5               g218(.a(\a[31] ), .b(\b[30] ), .out0(new_n314));
  oaih12aa1n02x5               g219(.a(new_n311), .b(new_n281), .c(new_n279), .o1(new_n315));
  norb02aa1n02x5               g220(.a(new_n314), .b(new_n312), .out0(new_n316));
  tech160nm_finand02aa1n03p5x5 g221(.a(new_n315), .b(new_n316), .o1(new_n317));
  oaih12aa1n02x5               g222(.a(new_n317), .b(new_n313), .c(new_n314), .o1(\s[31] ));
  xnrb03aa1n02x5               g223(.a(new_n115), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanb02aa1n02x5               g224(.a(new_n116), .b(new_n117), .out0(new_n320));
  oaib12aa1n02x5               g225(.a(new_n119), .b(new_n118), .c(new_n115), .out0(new_n321));
  aboi22aa1n03x5               g226(.a(new_n116), .b(new_n122), .c(new_n321), .d(new_n320), .out0(\s[4] ));
  oaoi13aa1n06x5               g227(.a(new_n124), .b(new_n121), .c(new_n120), .d(new_n115), .o1(new_n323));
  oai112aa1n02x5               g228(.a(new_n121), .b(new_n124), .c(new_n120), .d(new_n115), .o1(new_n324));
  norb02aa1n02x5               g229(.a(new_n324), .b(new_n323), .out0(\s[5] ));
  oabi12aa1n02x5               g230(.a(new_n123), .b(new_n323), .c(new_n109), .out0(new_n326));
  norb03aa1n02x5               g231(.a(new_n123), .b(new_n323), .c(new_n109), .out0(new_n327));
  norb02aa1n02x5               g232(.a(new_n326), .b(new_n327), .out0(\s[6] ));
  nanb02aa1n02x5               g233(.a(new_n104), .b(new_n105), .out0(new_n329));
  nanp02aa1n02x5               g234(.a(new_n108), .b(new_n107), .o1(new_n330));
  xobna2aa1n03x5               g235(.a(new_n329), .b(new_n326), .c(new_n330), .out0(\s[7] ));
  norb03aa1n02x5               g236(.a(new_n102), .b(new_n104), .c(new_n103), .out0(new_n332));
  tech160nm_fiaoi012aa1n05x5   g237(.a(new_n329), .b(new_n326), .c(new_n330), .o1(new_n333));
  obai22aa1n02x7               g238(.a(new_n102), .b(new_n103), .c(new_n333), .d(new_n104), .out0(new_n334));
  oaib12aa1n02x5               g239(.a(new_n334), .b(new_n333), .c(new_n332), .out0(\s[8] ));
  nanb02aa1n02x5               g240(.a(new_n101), .b(new_n100), .out0(new_n336));
  nano23aa1n02x4               g241(.a(new_n111), .b(new_n127), .c(new_n126), .d(new_n336), .out0(new_n337));
  aoib12aa1n02x5               g242(.a(new_n337), .b(new_n160), .c(new_n336), .out0(\s[9] ));
endmodule


