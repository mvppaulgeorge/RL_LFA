// Benchmark "adder" written by ABC on Thu Jul 18 05:09:55 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n123, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n147, new_n148, new_n149,
    new_n150, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n162, new_n163, new_n164, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n312, new_n314, new_n315, new_n316, new_n318,
    new_n320, new_n322, new_n324;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  oai022aa1n02x7               g001(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n97));
  xnrc02aa1n12x5               g002(.a(\b[2] ), .b(\a[3] ), .out0(new_n98));
  nor042aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n09x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand22aa1n12x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n12x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  oabi12aa1n18x5               g007(.a(new_n97), .b(new_n98), .c(new_n102), .out0(new_n103));
  nand42aa1n08x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nor002aa1d32x5               g009(.a(\b[7] ), .b(\a[8] ), .o1(new_n105));
  nor002aa1d32x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  norb03aa1n03x5               g011(.a(new_n104), .b(new_n106), .c(new_n105), .out0(new_n107));
  nand22aa1n06x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  orn002aa1n12x5               g013(.a(\a[6] ), .b(\b[5] ), .o(new_n109));
  nanp02aa1n09x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand42aa1n03x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nanb03aa1n06x5               g018(.a(new_n112), .b(new_n113), .c(new_n111), .out0(new_n114));
  nano32aa1n03x7               g019(.a(new_n114), .b(new_n109), .c(new_n110), .d(new_n108), .out0(new_n115));
  aob012aa1n02x5               g020(.a(new_n109), .b(new_n112), .c(new_n108), .out0(new_n116));
  nona23aa1n09x5               g021(.a(new_n104), .b(new_n110), .c(new_n106), .d(new_n105), .out0(new_n117));
  oai012aa1n02x5               g022(.a(new_n110), .b(new_n106), .c(new_n105), .o1(new_n118));
  oaib12aa1n02x7               g023(.a(new_n118), .b(new_n117), .c(new_n116), .out0(new_n119));
  aoi013aa1n06x4               g024(.a(new_n119), .b(new_n103), .c(new_n107), .d(new_n115), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[9] ), .b(\b[8] ), .c(new_n120), .o1(new_n121));
  xorb03aa1n02x5               g026(.a(new_n121), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand23aa1n09x5               g027(.a(new_n103), .b(new_n107), .c(new_n115), .o1(new_n123));
  oai022aa1n02x5               g028(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n124));
  aboi22aa1n06x5               g029(.a(new_n117), .b(new_n116), .c(new_n110), .d(new_n124), .out0(new_n125));
  nand02aa1n08x5               g030(.a(new_n123), .b(new_n125), .o1(new_n126));
  aob012aa1n06x5               g031(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(new_n127));
  oai022aa1n02x7               g032(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n128));
  aboi22aa1n12x5               g033(.a(new_n128), .b(new_n127), .c(\b[9] ), .d(\a[10] ), .out0(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n08x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  xorc02aa1n12x5               g036(.a(\a[11] ), .b(\b[10] ), .out0(new_n132));
  tech160nm_fixnrc02aa1n05x5   g037(.a(\b[11] ), .b(\a[12] ), .out0(new_n133));
  aoai13aa1n03x5               g038(.a(new_n133), .b(new_n131), .c(new_n129), .d(new_n132), .o1(new_n134));
  aoi112aa1n03x5               g039(.a(new_n133), .b(new_n131), .c(new_n129), .d(new_n132), .o1(new_n135));
  nanb02aa1n03x5               g040(.a(new_n135), .b(new_n134), .out0(\s[12] ));
  xnrc02aa1n02x5               g041(.a(\b[9] ), .b(\a[10] ), .out0(new_n137));
  tech160nm_fixorc02aa1n05x5   g042(.a(\a[9] ), .b(\b[8] ), .out0(new_n138));
  nona23aa1n09x5               g043(.a(new_n138), .b(new_n132), .c(new_n137), .d(new_n133), .out0(new_n139));
  nanp02aa1n06x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  oab012aa1n02x4               g045(.a(new_n131), .b(\a[12] ), .c(\b[11] ), .out0(new_n141));
  aoi022aa1n02x5               g046(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n142));
  aob012aa1n06x5               g047(.a(new_n141), .b(new_n128), .c(new_n142), .out0(new_n143));
  nanp02aa1n02x5               g048(.a(new_n143), .b(new_n140), .o1(new_n144));
  aoai13aa1n06x5               g049(.a(new_n144), .b(new_n139), .c(new_n123), .d(new_n125), .o1(new_n145));
  xorb03aa1n02x5               g050(.a(new_n145), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv040aa1d28x5               g051(.a(\a[14] ), .o1(new_n147));
  inv040aa1d32x5               g052(.a(\a[13] ), .o1(new_n148));
  inv000aa1d42x5               g053(.a(\b[12] ), .o1(new_n149));
  oaoi03aa1n02x5               g054(.a(new_n148), .b(new_n149), .c(new_n145), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[13] ), .c(new_n147), .out0(\s[14] ));
  xroi22aa1d06x4               g056(.a(new_n148), .b(\b[12] ), .c(new_n147), .d(\b[13] ), .out0(new_n152));
  nor042aa1n06x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  inv040aa1n03x5               g058(.a(new_n153), .o1(new_n154));
  oaoi03aa1n09x5               g059(.a(\a[14] ), .b(\b[13] ), .c(new_n154), .o1(new_n155));
  nor042aa1n03x5               g060(.a(\b[14] ), .b(\a[15] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n155), .c(new_n145), .d(new_n152), .o1(new_n159));
  aoi112aa1n02x5               g064(.a(new_n158), .b(new_n155), .c(new_n145), .d(new_n152), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n159), .b(new_n160), .out0(\s[15] ));
  inv000aa1n02x5               g066(.a(new_n156), .o1(new_n162));
  xnrc02aa1n12x5               g067(.a(\b[15] ), .b(\a[16] ), .out0(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  xnbna2aa1n03x5               g069(.a(new_n164), .b(new_n159), .c(new_n162), .out0(\s[16] ));
  xorc02aa1n02x5               g070(.a(\a[12] ), .b(\b[11] ), .out0(new_n166));
  nanp02aa1n02x5               g071(.a(new_n166), .b(new_n132), .o1(new_n167));
  xorc02aa1n02x5               g072(.a(\a[10] ), .b(\b[9] ), .out0(new_n168));
  nanp02aa1n02x5               g073(.a(new_n138), .b(new_n168), .o1(new_n169));
  nano22aa1n03x7               g074(.a(new_n163), .b(new_n162), .c(new_n157), .out0(new_n170));
  nona23aa1n09x5               g075(.a(new_n152), .b(new_n170), .c(new_n169), .d(new_n167), .out0(new_n171));
  xnrc02aa1n02x5               g076(.a(\b[13] ), .b(\a[14] ), .out0(new_n172));
  oai012aa1n02x5               g077(.a(new_n140), .b(\b[12] ), .c(\a[13] ), .o1(new_n173));
  oaih22aa1n04x5               g078(.a(new_n148), .b(new_n149), .c(\b[14] ), .d(\a[15] ), .o1(new_n174));
  nor043aa1n06x5               g079(.a(new_n172), .b(new_n173), .c(new_n174), .o1(new_n175));
  norb02aa1n09x5               g080(.a(new_n157), .b(new_n163), .out0(new_n176));
  aoi022aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n177));
  tech160nm_fioai012aa1n05x5   g082(.a(new_n177), .b(new_n155), .c(new_n156), .o1(new_n178));
  oai012aa1n02x5               g083(.a(new_n178), .b(\b[15] ), .c(\a[16] ), .o1(new_n179));
  aoi013aa1n06x4               g084(.a(new_n179), .b(new_n175), .c(new_n143), .d(new_n176), .o1(new_n180));
  oai012aa1n06x5               g085(.a(new_n180), .b(new_n120), .c(new_n171), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1d18x5               g087(.a(\b[16] ), .b(\a[17] ), .o1(new_n183));
  nand02aa1d12x5               g088(.a(\b[16] ), .b(\a[17] ), .o1(new_n184));
  tech160nm_fiaoi012aa1n05x5   g089(.a(new_n183), .b(new_n181), .c(new_n184), .o1(new_n185));
  xnrb03aa1n03x5               g090(.a(new_n185), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nano22aa1n12x5               g091(.a(new_n139), .b(new_n152), .c(new_n170), .out0(new_n187));
  nanp03aa1n06x5               g092(.a(new_n175), .b(new_n143), .c(new_n176), .o1(new_n188));
  oai112aa1n06x5               g093(.a(new_n188), .b(new_n178), .c(\b[15] ), .d(\a[16] ), .o1(new_n189));
  nor042aa1n12x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nand02aa1d28x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  nano23aa1n06x5               g096(.a(new_n183), .b(new_n190), .c(new_n191), .d(new_n184), .out0(new_n192));
  aoai13aa1n06x5               g097(.a(new_n192), .b(new_n189), .c(new_n126), .d(new_n187), .o1(new_n193));
  aoi012aa1d24x5               g098(.a(new_n190), .b(new_n183), .c(new_n191), .o1(new_n194));
  nor002aa1n20x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nand42aa1d28x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n193), .c(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aobi12aa1n02x7               g104(.a(new_n197), .b(new_n193), .c(new_n194), .out0(new_n200));
  nor002aa1d32x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nand42aa1d28x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  oabi12aa1n02x5               g108(.a(new_n203), .b(new_n200), .c(new_n195), .out0(new_n204));
  inv020aa1n04x5               g109(.a(new_n194), .o1(new_n205));
  aoai13aa1n03x5               g110(.a(new_n197), .b(new_n205), .c(new_n181), .d(new_n192), .o1(new_n206));
  oai112aa1n03x5               g111(.a(new_n206), .b(new_n203), .c(\b[18] ), .d(\a[19] ), .o1(new_n207));
  nanp02aa1n03x5               g112(.a(new_n204), .b(new_n207), .o1(\s[20] ));
  tech160nm_fiaoi012aa1n05x5   g113(.a(new_n189), .b(new_n126), .c(new_n187), .o1(new_n209));
  nona23aa1n09x5               g114(.a(new_n202), .b(new_n196), .c(new_n195), .d(new_n201), .out0(new_n210));
  norb02aa1n09x5               g115(.a(new_n192), .b(new_n210), .out0(new_n211));
  inv000aa1n02x5               g116(.a(new_n211), .o1(new_n212));
  tech160nm_fioai012aa1n03p5x5 g117(.a(new_n202), .b(new_n201), .c(new_n195), .o1(new_n213));
  oai012aa1n04x7               g118(.a(new_n213), .b(new_n210), .c(new_n194), .o1(new_n214));
  oabi12aa1n06x5               g119(.a(new_n214), .b(new_n209), .c(new_n212), .out0(new_n215));
  xorb03aa1n02x5               g120(.a(new_n215), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n12x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xnrc02aa1n12x5               g123(.a(\b[21] ), .b(\a[22] ), .out0(new_n219));
  aoai13aa1n03x5               g124(.a(new_n219), .b(new_n217), .c(new_n215), .d(new_n218), .o1(new_n220));
  aoai13aa1n03x5               g125(.a(new_n218), .b(new_n214), .c(new_n181), .d(new_n211), .o1(new_n221));
  nona22aa1n02x4               g126(.a(new_n221), .b(new_n219), .c(new_n217), .out0(new_n222));
  nanp02aa1n03x5               g127(.a(new_n220), .b(new_n222), .o1(\s[22] ));
  nano23aa1n09x5               g128(.a(new_n195), .b(new_n201), .c(new_n202), .d(new_n196), .out0(new_n224));
  nanb02aa1n12x5               g129(.a(new_n219), .b(new_n218), .out0(new_n225));
  nano22aa1n03x7               g130(.a(new_n225), .b(new_n192), .c(new_n224), .out0(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n189), .c(new_n126), .d(new_n187), .o1(new_n227));
  nand02aa1d06x5               g132(.a(new_n224), .b(new_n205), .o1(new_n228));
  inv020aa1d32x5               g133(.a(\a[22] ), .o1(new_n229));
  inv040aa1d32x5               g134(.a(\b[21] ), .o1(new_n230));
  oao003aa1n03x5               g135(.a(new_n229), .b(new_n230), .c(new_n217), .carry(new_n231));
  inv040aa1n03x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n12x5               g137(.a(new_n232), .b(new_n225), .c(new_n228), .d(new_n213), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  xorc02aa1n12x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  xnbna2aa1n03x5               g140(.a(new_n235), .b(new_n227), .c(new_n234), .out0(\s[23] ));
  nand42aa1n02x5               g141(.a(new_n227), .b(new_n234), .o1(new_n237));
  nor042aa1n03x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  xnrc02aa1n12x5               g143(.a(\b[23] ), .b(\a[24] ), .out0(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n238), .c(new_n237), .d(new_n235), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n235), .b(new_n233), .c(new_n181), .d(new_n226), .o1(new_n241));
  nona22aa1n03x5               g146(.a(new_n241), .b(new_n239), .c(new_n238), .out0(new_n242));
  nanp02aa1n03x5               g147(.a(new_n240), .b(new_n242), .o1(\s[24] ));
  norb02aa1n03x4               g148(.a(new_n218), .b(new_n219), .out0(new_n244));
  norb02aa1n03x5               g149(.a(new_n235), .b(new_n239), .out0(new_n245));
  nano22aa1n02x4               g150(.a(new_n212), .b(new_n244), .c(new_n245), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n189), .c(new_n126), .d(new_n187), .o1(new_n247));
  aoai13aa1n04x5               g152(.a(new_n245), .b(new_n231), .c(new_n214), .d(new_n244), .o1(new_n248));
  inv000aa1d42x5               g153(.a(\a[24] ), .o1(new_n249));
  inv000aa1d42x5               g154(.a(\b[23] ), .o1(new_n250));
  oao003aa1n02x5               g155(.a(new_n249), .b(new_n250), .c(new_n238), .carry(new_n251));
  inv000aa1n02x5               g156(.a(new_n251), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(new_n248), .b(new_n252), .o1(new_n253));
  nanb02aa1n06x5               g158(.a(new_n253), .b(new_n247), .out0(new_n254));
  xorb03aa1n02x5               g159(.a(new_n254), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n12x5               g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  tech160nm_fixnrc02aa1n05x5   g162(.a(\b[25] ), .b(\a[26] ), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n258), .b(new_n256), .c(new_n254), .d(new_n257), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n257), .b(new_n253), .c(new_n181), .d(new_n246), .o1(new_n260));
  nona22aa1n03x5               g165(.a(new_n260), .b(new_n258), .c(new_n256), .out0(new_n261));
  nanp02aa1n03x5               g166(.a(new_n259), .b(new_n261), .o1(\s[26] ));
  nanb02aa1n03x5               g167(.a(new_n258), .b(new_n257), .out0(new_n263));
  inv000aa1n03x5               g168(.a(new_n263), .o1(new_n264));
  nand03aa1n02x5               g169(.a(new_n226), .b(new_n245), .c(new_n264), .o1(new_n265));
  inv000aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n189), .c(new_n126), .d(new_n187), .o1(new_n267));
  aoai13aa1n12x5               g172(.a(new_n264), .b(new_n251), .c(new_n233), .d(new_n245), .o1(new_n268));
  oai022aa1n02x5               g173(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n269));
  aob012aa1n02x5               g174(.a(new_n269), .b(\b[25] ), .c(\a[26] ), .out0(new_n270));
  nand23aa1n03x5               g175(.a(new_n267), .b(new_n268), .c(new_n270), .o1(new_n271));
  xorb03aa1n02x5               g176(.a(new_n271), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  xorc02aa1n12x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  norp02aa1n03x5               g179(.a(\b[27] ), .b(\a[28] ), .o1(new_n275));
  nand42aa1n03x5               g180(.a(\b[27] ), .b(\a[28] ), .o1(new_n276));
  nanb02aa1n09x5               g181(.a(new_n275), .b(new_n276), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n273), .c(new_n271), .d(new_n274), .o1(new_n278));
  oaoi13aa1n06x5               g183(.a(new_n265), .b(new_n180), .c(new_n120), .d(new_n171), .o1(new_n279));
  aoai13aa1n04x5               g184(.a(new_n270), .b(new_n263), .c(new_n248), .d(new_n252), .o1(new_n280));
  oaih12aa1n02x5               g185(.a(new_n274), .b(new_n280), .c(new_n279), .o1(new_n281));
  nona22aa1n02x4               g186(.a(new_n281), .b(new_n277), .c(new_n273), .out0(new_n282));
  nanp02aa1n03x5               g187(.a(new_n278), .b(new_n282), .o1(\s[28] ));
  norb02aa1n02x7               g188(.a(new_n274), .b(new_n277), .out0(new_n284));
  oaih12aa1n02x5               g189(.a(new_n284), .b(new_n280), .c(new_n279), .o1(new_n285));
  aoi012aa1n02x5               g190(.a(new_n275), .b(new_n273), .c(new_n276), .o1(new_n286));
  tech160nm_fixnrc02aa1n04x5   g191(.a(\b[28] ), .b(\a[29] ), .out0(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(new_n285), .c(new_n286), .o1(new_n288));
  inv000aa1n02x5               g193(.a(new_n284), .o1(new_n289));
  aoi013aa1n03x5               g194(.a(new_n289), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n290));
  nano22aa1n03x5               g195(.a(new_n290), .b(new_n286), .c(new_n287), .out0(new_n291));
  norp02aa1n03x5               g196(.a(new_n288), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  norb03aa1n06x5               g199(.a(new_n274), .b(new_n287), .c(new_n277), .out0(new_n295));
  oaih12aa1n02x5               g200(.a(new_n295), .b(new_n280), .c(new_n279), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n286), .carry(new_n297));
  tech160nm_fiaoi012aa1n02p5x5 g202(.a(new_n294), .b(new_n296), .c(new_n297), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n295), .o1(new_n299));
  aoi013aa1n03x5               g204(.a(new_n299), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n300));
  nano22aa1n03x5               g205(.a(new_n300), .b(new_n294), .c(new_n297), .out0(new_n301));
  norp02aa1n03x5               g206(.a(new_n298), .b(new_n301), .o1(\s[30] ));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  norb02aa1n02x5               g208(.a(new_n295), .b(new_n294), .out0(new_n304));
  oaih12aa1n02x5               g209(.a(new_n304), .b(new_n280), .c(new_n279), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n297), .carry(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n303), .b(new_n305), .c(new_n306), .o1(new_n307));
  inv000aa1n02x5               g212(.a(new_n304), .o1(new_n308));
  aoi013aa1n03x5               g213(.a(new_n308), .b(new_n267), .c(new_n268), .d(new_n270), .o1(new_n309));
  nano22aa1n03x5               g214(.a(new_n309), .b(new_n303), .c(new_n306), .out0(new_n310));
  norp02aa1n03x5               g215(.a(new_n307), .b(new_n310), .o1(\s[31] ));
  inv000aa1d42x5               g216(.a(\a[3] ), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n102), .b(\b[2] ), .c(new_n312), .out0(\s[3] ));
  norp02aa1n02x5               g218(.a(new_n98), .b(new_n102), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[4] ), .b(\b[3] ), .out0(new_n315));
  aoib12aa1n02x5               g220(.a(new_n315), .b(new_n312), .c(\b[2] ), .out0(new_n316));
  aboi22aa1n03x5               g221(.a(new_n314), .b(new_n316), .c(new_n103), .d(new_n315), .out0(\s[4] ));
  norb02aa1n02x5               g222(.a(new_n113), .b(new_n112), .out0(new_n318));
  xobna2aa1n03x5               g223(.a(new_n318), .b(new_n103), .c(new_n111), .out0(\s[5] ));
  aoib12aa1n06x5               g224(.a(new_n112), .b(new_n103), .c(new_n114), .out0(new_n320));
  xnbna2aa1n03x5               g225(.a(new_n320), .b(new_n108), .c(new_n109), .out0(\s[6] ));
  oaoi03aa1n02x5               g226(.a(\a[6] ), .b(\b[5] ), .c(new_n320), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n03x5               g228(.a(new_n106), .b(new_n322), .c(new_n104), .o1(new_n324));
  xnrb03aa1n02x5               g229(.a(new_n324), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g230(.a(new_n138), .b(new_n123), .c(new_n125), .out0(\s[9] ));
endmodule


