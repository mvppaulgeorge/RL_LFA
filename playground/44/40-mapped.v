// Benchmark "adder" written by ABC on Thu Jul 18 10:53:52 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n147, new_n148, new_n149,
    new_n151, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n160, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n211, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n323, new_n325, new_n326, new_n327;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1d04x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nand22aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1n12x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand22aa1n06x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor022aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n03x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  ao0012aa1n03x7               g010(.a(new_n101), .b(new_n103), .c(new_n102), .o(new_n106));
  oabi12aa1n09x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .out0(new_n107));
  nand02aa1n16x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nor002aa1n12x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nand02aa1d10x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanb03aa1n06x5               g015(.a(new_n109), .b(new_n110), .c(new_n108), .out0(new_n111));
  xnrc02aa1n12x5               g016(.a(\b[7] ), .b(\a[8] ), .out0(new_n112));
  norp02aa1n03x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nona22aa1n06x5               g020(.a(new_n115), .b(new_n114), .c(new_n113), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n116), .b(new_n111), .c(new_n112), .o1(new_n117));
  oaih22aa1d12x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  aoi013aa1n03x5               g023(.a(new_n109), .b(new_n118), .c(new_n110), .d(new_n108), .o1(new_n119));
  oaoi03aa1n09x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n119), .o1(new_n120));
  aoi012aa1n06x5               g025(.a(new_n120), .b(new_n107), .c(new_n117), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .c(new_n121), .o1(new_n122));
  xorb03aa1n02x5               g027(.a(new_n122), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp02aa1n06x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  nor022aa1n06x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nona23aa1n06x5               g032(.a(new_n127), .b(new_n125), .c(new_n124), .d(new_n126), .out0(new_n128));
  oaih12aa1n02x5               g033(.a(new_n127), .b(new_n126), .c(new_n124), .o1(new_n129));
  oai012aa1n02x5               g034(.a(new_n129), .b(new_n121), .c(new_n128), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n06x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand22aa1n12x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  aoi012aa1n03x5               g038(.a(new_n132), .b(new_n130), .c(new_n133), .o1(new_n134));
  nor002aa1n16x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanp02aa1n06x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  xnrc02aa1n02x5               g042(.a(new_n134), .b(new_n137), .out0(\s[12] ));
  nona23aa1n03x5               g043(.a(new_n136), .b(new_n133), .c(new_n132), .d(new_n135), .out0(new_n139));
  nor042aa1n03x5               g044(.a(new_n139), .b(new_n128), .o1(new_n140));
  aoai13aa1n06x5               g045(.a(new_n140), .b(new_n120), .c(new_n107), .d(new_n117), .o1(new_n141));
  nor002aa1n02x5               g046(.a(new_n135), .b(new_n132), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(new_n129), .b(new_n142), .o1(new_n143));
  oai012aa1n18x5               g048(.a(new_n136), .b(new_n135), .c(new_n133), .o1(new_n144));
  oaib12aa1n02x5               g049(.a(new_n141), .b(new_n144), .c(new_n143), .out0(new_n145));
  xorb03aa1n02x5               g050(.a(new_n145), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  nand02aa1d06x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  aoi012aa1n02x5               g053(.a(new_n147), .b(new_n145), .c(new_n148), .o1(new_n149));
  xnrb03aa1n02x5               g054(.a(new_n149), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n06x5               g055(.a(\b[13] ), .b(\a[14] ), .o1(new_n151));
  nand02aa1d06x5               g056(.a(\b[13] ), .b(\a[14] ), .o1(new_n152));
  nona23aa1n03x5               g057(.a(new_n152), .b(new_n148), .c(new_n147), .d(new_n151), .out0(new_n153));
  inv000aa1d42x5               g058(.a(new_n144), .o1(new_n154));
  nano23aa1n06x5               g059(.a(new_n147), .b(new_n151), .c(new_n152), .d(new_n148), .out0(new_n155));
  nanp03aa1n02x5               g060(.a(new_n143), .b(new_n155), .c(new_n154), .o1(new_n156));
  aoi012aa1n02x7               g061(.a(new_n151), .b(new_n147), .c(new_n152), .o1(new_n157));
  oai112aa1n03x5               g062(.a(new_n156), .b(new_n157), .c(new_n141), .d(new_n153), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  xnrc02aa1n12x5               g065(.a(\b[14] ), .b(\a[15] ), .out0(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  tech160nm_fixnrc02aa1n04x5   g067(.a(\b[15] ), .b(\a[16] ), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n160), .c(new_n158), .d(new_n162), .o1(new_n164));
  aoi112aa1n03x4               g069(.a(new_n160), .b(new_n163), .c(new_n158), .d(new_n162), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(\s[16] ));
  nor042aa1n03x5               g071(.a(new_n163), .b(new_n161), .o1(new_n167));
  nona23aa1d18x5               g072(.a(new_n155), .b(new_n167), .c(new_n139), .d(new_n128), .out0(new_n168));
  aoi112aa1n03x5               g073(.a(new_n153), .b(new_n144), .c(new_n129), .d(new_n142), .o1(new_n169));
  inv000aa1n02x5               g074(.a(new_n157), .o1(new_n170));
  inv000aa1d42x5               g075(.a(\a[16] ), .o1(new_n171));
  inv000aa1d42x5               g076(.a(\b[15] ), .o1(new_n172));
  oaoi03aa1n09x5               g077(.a(new_n171), .b(new_n172), .c(new_n160), .o1(new_n173));
  inv000aa1n02x5               g078(.a(new_n173), .o1(new_n174));
  oaoi13aa1n12x5               g079(.a(new_n174), .b(new_n167), .c(new_n169), .d(new_n170), .o1(new_n175));
  oai012aa1n12x5               g080(.a(new_n175), .b(new_n121), .c(new_n168), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g082(.a(\a[18] ), .o1(new_n178));
  inv040aa1d30x5               g083(.a(\a[17] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[16] ), .o1(new_n180));
  oaoi03aa1n03x5               g085(.a(new_n179), .b(new_n180), .c(new_n176), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(new_n178), .out0(\s[18] ));
  nona22aa1n03x5               g087(.a(new_n155), .b(new_n161), .c(new_n163), .out0(new_n183));
  norb02aa1n06x5               g088(.a(new_n140), .b(new_n183), .out0(new_n184));
  aoai13aa1n06x5               g089(.a(new_n184), .b(new_n120), .c(new_n107), .d(new_n117), .o1(new_n185));
  xroi22aa1d06x4               g090(.a(new_n179), .b(\b[16] ), .c(new_n178), .d(\b[17] ), .out0(new_n186));
  inv000aa1d42x5               g091(.a(new_n186), .o1(new_n187));
  nor002aa1n12x5               g092(.a(\b[17] ), .b(\a[18] ), .o1(new_n188));
  nand42aa1n04x5               g093(.a(\b[17] ), .b(\a[18] ), .o1(new_n189));
  aoai13aa1n12x5               g094(.a(new_n189), .b(new_n188), .c(new_n179), .d(new_n180), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n187), .c(new_n185), .d(new_n175), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g097(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n06x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nand22aa1n09x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nor042aa1n09x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  nand42aa1n02x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  nanb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(new_n198));
  aoai13aa1n03x5               g103(.a(new_n198), .b(new_n194), .c(new_n191), .d(new_n195), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n194), .b(new_n195), .out0(new_n200));
  nanb02aa1n03x5               g105(.a(new_n200), .b(new_n191), .out0(new_n201));
  nona22aa1n02x5               g106(.a(new_n201), .b(new_n198), .c(new_n194), .out0(new_n202));
  nanp02aa1n03x5               g107(.a(new_n202), .b(new_n199), .o1(\s[20] ));
  nona22aa1n03x5               g108(.a(new_n186), .b(new_n200), .c(new_n198), .out0(new_n204));
  nor042aa1n03x5               g109(.a(new_n196), .b(new_n194), .o1(new_n205));
  oai012aa1n06x5               g110(.a(new_n197), .b(new_n196), .c(new_n195), .o1(new_n206));
  aoi012aa1n12x5               g111(.a(new_n206), .b(new_n190), .c(new_n205), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n03x5               g113(.a(new_n208), .b(new_n204), .c(new_n185), .d(new_n175), .o1(new_n209));
  xorb03aa1n02x5               g114(.a(new_n209), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g115(.a(\b[20] ), .b(\a[21] ), .o1(new_n211));
  nand02aa1d04x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  nor042aa1n04x5               g118(.a(\b[21] ), .b(\a[22] ), .o1(new_n214));
  nand02aa1d08x5               g119(.a(\b[21] ), .b(\a[22] ), .o1(new_n215));
  nanb02aa1n02x5               g120(.a(new_n214), .b(new_n215), .out0(new_n216));
  aoai13aa1n02x5               g121(.a(new_n216), .b(new_n211), .c(new_n209), .d(new_n213), .o1(new_n217));
  inv040aa1n02x5               g122(.a(new_n204), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n213), .b(new_n207), .c(new_n176), .d(new_n218), .o1(new_n219));
  nona22aa1n02x4               g124(.a(new_n219), .b(new_n216), .c(new_n211), .out0(new_n220));
  nanp02aa1n03x5               g125(.a(new_n217), .b(new_n220), .o1(\s[22] ));
  nona23aa1n02x4               g126(.a(new_n215), .b(new_n212), .c(new_n211), .d(new_n214), .out0(new_n222));
  nona32aa1n03x5               g127(.a(new_n186), .b(new_n222), .c(new_n198), .d(new_n200), .out0(new_n223));
  nand42aa1n04x5               g128(.a(new_n190), .b(new_n205), .o1(new_n224));
  inv020aa1n04x5               g129(.a(new_n206), .o1(new_n225));
  nano23aa1n06x5               g130(.a(new_n211), .b(new_n214), .c(new_n215), .d(new_n212), .out0(new_n226));
  aoi012aa1d18x5               g131(.a(new_n214), .b(new_n211), .c(new_n215), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoi013aa1n02x4               g133(.a(new_n228), .b(new_n224), .c(new_n226), .d(new_n225), .o1(new_n229));
  aoai13aa1n04x5               g134(.a(new_n229), .b(new_n223), .c(new_n185), .d(new_n175), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n12x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  tech160nm_fixnrc02aa1n05x5   g138(.a(\b[23] ), .b(\a[24] ), .out0(new_n234));
  aoai13aa1n03x5               g139(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n235));
  nand02aa1n02x5               g140(.a(new_n230), .b(new_n233), .o1(new_n236));
  nona22aa1n02x4               g141(.a(new_n236), .b(new_n234), .c(new_n232), .out0(new_n237));
  nanp02aa1n03x5               g142(.a(new_n237), .b(new_n235), .o1(\s[24] ));
  norb02aa1n03x4               g143(.a(new_n233), .b(new_n234), .out0(new_n239));
  inv000aa1n03x5               g144(.a(new_n239), .o1(new_n240));
  nona22aa1n02x4               g145(.a(new_n218), .b(new_n240), .c(new_n222), .out0(new_n241));
  nand23aa1n03x5               g146(.a(new_n224), .b(new_n226), .c(new_n225), .o1(new_n242));
  orn002aa1n02x5               g147(.a(\a[23] ), .b(\b[22] ), .o(new_n243));
  oao003aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .c(new_n243), .carry(new_n244));
  aoai13aa1n12x5               g149(.a(new_n244), .b(new_n240), .c(new_n242), .d(new_n227), .o1(new_n245));
  inv000aa1n02x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n04x5               g151(.a(new_n246), .b(new_n241), .c(new_n185), .d(new_n175), .o1(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g153(.a(\b[24] ), .b(\a[25] ), .o1(new_n249));
  xorc02aa1n03x5               g154(.a(\a[25] ), .b(\b[24] ), .out0(new_n250));
  xnrc02aa1n12x5               g155(.a(\b[25] ), .b(\a[26] ), .out0(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n249), .c(new_n247), .d(new_n250), .o1(new_n252));
  nand02aa1n02x5               g157(.a(new_n247), .b(new_n250), .o1(new_n253));
  nona22aa1n02x4               g158(.a(new_n253), .b(new_n251), .c(new_n249), .out0(new_n254));
  nanp02aa1n03x5               g159(.a(new_n254), .b(new_n252), .o1(\s[26] ));
  inv000aa1n02x5               g160(.a(new_n100), .o1(new_n256));
  nano23aa1n02x4               g161(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n257));
  aoi012aa1n02x5               g162(.a(new_n106), .b(new_n257), .c(new_n256), .o1(new_n258));
  inv000aa1d42x5               g163(.a(\a[8] ), .o1(new_n259));
  inv000aa1d42x5               g164(.a(\b[7] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n108), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n109), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n110), .o1(new_n263));
  nor022aa1n02x5               g168(.a(new_n114), .b(new_n113), .o1(new_n264));
  oai013aa1n02x4               g169(.a(new_n262), .b(new_n264), .c(new_n263), .d(new_n261), .o1(new_n265));
  oaoi03aa1n02x5               g170(.a(new_n259), .b(new_n260), .c(new_n265), .o1(new_n266));
  oaib12aa1n02x5               g171(.a(new_n266), .b(new_n258), .c(new_n117), .out0(new_n267));
  inv000aa1n02x5               g172(.a(new_n167), .o1(new_n268));
  aoai13aa1n02x5               g173(.a(new_n173), .b(new_n268), .c(new_n156), .d(new_n157), .o1(new_n269));
  norb02aa1n06x5               g174(.a(new_n250), .b(new_n251), .out0(new_n270));
  nano22aa1n03x7               g175(.a(new_n223), .b(new_n239), .c(new_n270), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n269), .c(new_n267), .d(new_n184), .o1(new_n272));
  orn002aa1n02x5               g177(.a(\a[25] ), .b(\b[24] ), .o(new_n273));
  oao003aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .c(new_n273), .carry(new_n274));
  aobi12aa1n12x5               g179(.a(new_n274), .b(new_n245), .c(new_n270), .out0(new_n275));
  xorc02aa1n12x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnbna2aa1n03x5               g181(.a(new_n276), .b(new_n272), .c(new_n275), .out0(\s[27] ));
  nand42aa1n02x5               g182(.a(new_n272), .b(new_n275), .o1(new_n278));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  norp02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .o1(new_n280));
  nand42aa1n06x5               g185(.a(\b[27] ), .b(\a[28] ), .o1(new_n281));
  norb02aa1n06x4               g186(.a(new_n281), .b(new_n280), .out0(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n283), .b(new_n279), .c(new_n278), .d(new_n276), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n239), .b(new_n228), .c(new_n207), .d(new_n226), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n270), .o1(new_n286));
  aoai13aa1n04x5               g191(.a(new_n274), .b(new_n286), .c(new_n285), .d(new_n244), .o1(new_n287));
  aoai13aa1n02x5               g192(.a(new_n276), .b(new_n287), .c(new_n176), .d(new_n271), .o1(new_n288));
  nona22aa1n02x4               g193(.a(new_n288), .b(new_n283), .c(new_n279), .out0(new_n289));
  nanp02aa1n03x5               g194(.a(new_n284), .b(new_n289), .o1(\s[28] ));
  norb02aa1n02x5               g195(.a(new_n276), .b(new_n283), .out0(new_n291));
  aoai13aa1n02x7               g196(.a(new_n291), .b(new_n287), .c(new_n176), .d(new_n271), .o1(new_n292));
  aoi012aa1n02x5               g197(.a(new_n280), .b(new_n279), .c(new_n281), .o1(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  aoi012aa1n02x5               g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  inv000aa1n02x5               g200(.a(new_n291), .o1(new_n296));
  tech160nm_fiaoi012aa1n02p5x5 g201(.a(new_n296), .b(new_n272), .c(new_n275), .o1(new_n297));
  nano22aa1n03x5               g202(.a(new_n297), .b(new_n293), .c(new_n294), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n295), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n09x5               g205(.a(new_n294), .b(new_n276), .c(new_n282), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n301), .b(new_n287), .c(new_n176), .d(new_n271), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[29] ), .b(\a[30] ), .out0(new_n304));
  aoi012aa1n02x7               g209(.a(new_n304), .b(new_n302), .c(new_n303), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n301), .o1(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n306), .b(new_n272), .c(new_n275), .o1(new_n307));
  nano22aa1n03x5               g212(.a(new_n307), .b(new_n303), .c(new_n304), .out0(new_n308));
  norp02aa1n03x5               g213(.a(new_n305), .b(new_n308), .o1(\s[30] ));
  nano23aa1n06x5               g214(.a(new_n304), .b(new_n294), .c(new_n276), .d(new_n282), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n287), .c(new_n176), .d(new_n271), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .carry(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[30] ), .b(\a[31] ), .out0(new_n313));
  aoi012aa1n02x5               g218(.a(new_n313), .b(new_n311), .c(new_n312), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n310), .o1(new_n315));
  tech160nm_fiaoi012aa1n02p5x5 g220(.a(new_n315), .b(new_n272), .c(new_n275), .o1(new_n316));
  nano22aa1n03x5               g221(.a(new_n316), .b(new_n312), .c(new_n313), .out0(new_n317));
  norp02aa1n03x5               g222(.a(new_n314), .b(new_n317), .o1(\s[31] ));
  xnrb03aa1n02x5               g223(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g224(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g226(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g227(.a(\a[5] ), .b(\b[4] ), .c(new_n258), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb03aa1n02x5               g229(.a(new_n115), .b(new_n258), .c(new_n114), .out0(new_n325));
  oabi12aa1n02x5               g230(.a(new_n111), .b(new_n325), .c(new_n118), .out0(new_n326));
  aoi122aa1n02x5               g231(.a(new_n113), .b(new_n262), .c(new_n110), .d(new_n323), .e(new_n108), .o1(new_n327));
  norb02aa1n02x5               g232(.a(new_n326), .b(new_n327), .out0(\s[7] ));
  xobna2aa1n03x5               g233(.a(new_n112), .b(new_n326), .c(new_n262), .out0(\s[8] ));
  xnrb03aa1n02x5               g234(.a(new_n121), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


