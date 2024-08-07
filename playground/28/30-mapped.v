// Benchmark "adder" written by ABC on Thu Jul 18 02:34:29 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n277, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n330, new_n332, new_n334;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor042aa1d18x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  xnrc02aa1n12x5               g006(.a(\b[7] ), .b(\a[8] ), .out0(new_n102));
  nor002aa1d32x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nanb02aa1n03x5               g009(.a(new_n103), .b(new_n104), .out0(new_n105));
  nand42aa1n02x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nor042aa1n09x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nor022aa1n16x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  tech160nm_fioai012aa1n03p5x5 g013(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n109));
  inv040aa1n03x5               g014(.a(new_n103), .o1(new_n110));
  oao003aa1n03x5               g015(.a(\a[8] ), .b(\b[7] ), .c(new_n110), .carry(new_n111));
  oai013aa1n03x5               g016(.a(new_n111), .b(new_n102), .c(new_n109), .d(new_n105), .o1(new_n112));
  nor022aa1n16x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nor002aa1n12x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nand02aa1n03x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  tech160nm_fiaoi012aa1n03p5x5 g020(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  nand22aa1n12x5               g022(.a(\b[0] ), .b(\a[1] ), .o1(new_n118));
  nor002aa1n02x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  tech160nm_fioai012aa1n05x5   g024(.a(new_n117), .b(new_n119), .c(new_n118), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[2] ), .b(\a[3] ), .o1(new_n121));
  nona23aa1n06x5               g026(.a(new_n115), .b(new_n121), .c(new_n114), .d(new_n113), .out0(new_n122));
  oai012aa1n12x5               g027(.a(new_n116), .b(new_n122), .c(new_n120), .o1(new_n123));
  nanb02aa1n02x5               g028(.a(new_n107), .b(new_n106), .out0(new_n124));
  nanp02aa1n02x5               g029(.a(\b[4] ), .b(\a[5] ), .o1(new_n125));
  nona23aa1n02x4               g030(.a(new_n125), .b(new_n104), .c(new_n103), .d(new_n108), .out0(new_n126));
  nor043aa1n06x5               g031(.a(new_n126), .b(new_n124), .c(new_n102), .o1(new_n127));
  nand02aa1d16x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n100), .out0(new_n129));
  aoai13aa1n03x5               g034(.a(new_n129), .b(new_n112), .c(new_n123), .d(new_n127), .o1(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n99), .b(new_n130), .c(new_n101), .out0(\s[10] ));
  inv040aa1d32x5               g036(.a(\a[11] ), .o1(new_n132));
  inv040aa1n12x5               g037(.a(\b[10] ), .o1(new_n133));
  nand22aa1n06x5               g038(.a(new_n133), .b(new_n132), .o1(new_n134));
  nand22aa1n12x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand02aa1d04x5               g040(.a(new_n134), .b(new_n135), .o1(new_n136));
  nona22aa1n02x4               g041(.a(new_n130), .b(new_n100), .c(new_n97), .out0(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n136), .b(new_n137), .c(new_n98), .out0(\s[11] ));
  inv040aa1d30x5               g043(.a(\a[12] ), .o1(new_n139));
  inv040aa1n15x5               g044(.a(\b[11] ), .o1(new_n140));
  nand02aa1n06x5               g045(.a(new_n140), .b(new_n139), .o1(new_n141));
  nand22aa1n04x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nor042aa1n04x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  aoi013aa1n03x5               g048(.a(new_n143), .b(new_n137), .c(new_n135), .d(new_n98), .o1(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n141), .c(new_n142), .out0(\s[12] ));
  tech160nm_fiaoi012aa1n05x5   g050(.a(new_n112), .b(new_n123), .c(new_n127), .o1(new_n146));
  nand22aa1n09x5               g051(.a(new_n141), .b(new_n142), .o1(new_n147));
  nano23aa1d15x5               g052(.a(new_n97), .b(new_n100), .c(new_n128), .d(new_n98), .out0(new_n148));
  nona22aa1d30x5               g053(.a(new_n148), .b(new_n147), .c(new_n136), .out0(new_n149));
  nanb03aa1n03x5               g054(.a(new_n143), .b(new_n135), .c(new_n98), .out0(new_n150));
  oai112aa1n06x5               g055(.a(new_n141), .b(new_n142), .c(new_n100), .d(new_n97), .o1(new_n151));
  tech160nm_fioaoi03aa1n03p5x5 g056(.a(new_n139), .b(new_n140), .c(new_n143), .o1(new_n152));
  tech160nm_fioai012aa1n05x5   g057(.a(new_n152), .b(new_n151), .c(new_n150), .o1(new_n153));
  oabi12aa1n02x5               g058(.a(new_n153), .b(new_n146), .c(new_n149), .out0(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nanp02aa1n06x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n156), .b(new_n154), .c(new_n157), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand22aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nano23aa1n06x5               g066(.a(new_n156), .b(new_n160), .c(new_n161), .d(new_n157), .out0(new_n162));
  oa0012aa1n02x5               g067(.a(new_n161), .b(new_n160), .c(new_n156), .o(new_n163));
  aoi012aa1n02x5               g068(.a(new_n163), .b(new_n153), .c(new_n162), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n149), .o1(new_n165));
  nano22aa1n03x7               g070(.a(new_n146), .b(new_n165), .c(new_n162), .out0(new_n166));
  nor042aa1n09x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand42aa1n04x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  oaib12aa1n06x5               g074(.a(new_n169), .b(new_n166), .c(new_n164), .out0(new_n170));
  norb03aa1n02x5               g075(.a(new_n164), .b(new_n166), .c(new_n169), .out0(new_n171));
  norb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(\s[15] ));
  inv000aa1d42x5               g077(.a(new_n167), .o1(new_n173));
  nor042aa1n04x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand02aa1n06x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n170), .c(new_n173), .out0(\s[16] ));
  nano23aa1d12x5               g082(.a(new_n167), .b(new_n174), .c(new_n175), .d(new_n168), .out0(new_n178));
  nano22aa1d15x5               g083(.a(new_n149), .b(new_n162), .c(new_n178), .out0(new_n179));
  aoai13aa1n12x5               g084(.a(new_n179), .b(new_n112), .c(new_n123), .d(new_n127), .o1(new_n180));
  aoai13aa1n06x5               g085(.a(new_n178), .b(new_n163), .c(new_n153), .d(new_n162), .o1(new_n181));
  aoi012aa1n12x5               g086(.a(new_n174), .b(new_n167), .c(new_n175), .o1(new_n182));
  nand23aa1d12x5               g087(.a(new_n180), .b(new_n181), .c(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g089(.a(\a[18] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\b[16] ), .o1(new_n187));
  oaoi03aa1n03x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  nona23aa1n02x4               g094(.a(new_n161), .b(new_n157), .c(new_n156), .d(new_n160), .out0(new_n190));
  oaoi13aa1n06x5               g095(.a(new_n190), .b(new_n152), .c(new_n151), .d(new_n150), .o1(new_n191));
  inv000aa1n02x5               g096(.a(new_n182), .o1(new_n192));
  oaoi13aa1n09x5               g097(.a(new_n192), .b(new_n178), .c(new_n191), .d(new_n163), .o1(new_n193));
  xroi22aa1d06x4               g098(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n194));
  inv000aa1n06x5               g099(.a(new_n194), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\b[17] ), .o1(new_n196));
  oai022aa1d18x5               g101(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n197));
  oa0012aa1n02x5               g102(.a(new_n197), .b(new_n196), .c(new_n185), .o(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  aoai13aa1n04x5               g104(.a(new_n199), .b(new_n195), .c(new_n193), .d(new_n180), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g107(.a(\a[19] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(\b[18] ), .o1(new_n204));
  nand22aa1n09x5               g109(.a(new_n204), .b(new_n203), .o1(new_n205));
  tech160nm_fixorc02aa1n04x5   g110(.a(\a[19] ), .b(\b[18] ), .out0(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n198), .c(new_n183), .d(new_n194), .o1(new_n207));
  xorc02aa1n12x5               g112(.a(\a[20] ), .b(\b[19] ), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  tech160nm_fiaoi012aa1n03p5x5 g114(.a(new_n209), .b(new_n207), .c(new_n205), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n205), .o1(new_n211));
  aoi112aa1n02x5               g116(.a(new_n211), .b(new_n208), .c(new_n200), .d(new_n206), .o1(new_n212));
  nor002aa1n02x5               g117(.a(new_n210), .b(new_n212), .o1(\s[20] ));
  nano22aa1n12x5               g118(.a(new_n195), .b(new_n206), .c(new_n208), .out0(new_n214));
  inv000aa1n02x5               g119(.a(new_n214), .o1(new_n215));
  and002aa1n02x5               g120(.a(\b[19] ), .b(\a[20] ), .o(new_n216));
  oai122aa1n06x5               g121(.a(new_n197), .b(new_n203), .c(new_n204), .d(new_n185), .e(new_n196), .o1(new_n217));
  oai112aa1n06x5               g122(.a(new_n217), .b(new_n205), .c(\b[19] ), .d(\a[20] ), .o1(new_n218));
  norb02aa1n15x5               g123(.a(new_n218), .b(new_n216), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n215), .c(new_n193), .d(new_n180), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  inv020aa1n03x5               g128(.a(new_n223), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[20] ), .b(\a[21] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n219), .c(new_n183), .d(new_n214), .o1(new_n227));
  xorc02aa1n12x5               g132(.a(\a[22] ), .b(\b[21] ), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  tech160nm_fiaoi012aa1n03p5x5 g134(.a(new_n229), .b(new_n227), .c(new_n224), .o1(new_n230));
  aoi112aa1n03x4               g135(.a(new_n223), .b(new_n228), .c(new_n221), .d(new_n226), .o1(new_n231));
  nor002aa1n02x5               g136(.a(new_n230), .b(new_n231), .o1(\s[22] ));
  xorc02aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  nona32aa1n03x5               g138(.a(new_n183), .b(new_n229), .c(new_n225), .d(new_n215), .out0(new_n234));
  inv000aa1d42x5               g139(.a(\a[22] ), .o1(new_n235));
  inv000aa1d42x5               g140(.a(\b[21] ), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(new_n236), .b(new_n235), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nano22aa1n02x4               g143(.a(new_n225), .b(new_n237), .c(new_n238), .out0(new_n239));
  oaoi03aa1n02x5               g144(.a(\a[22] ), .b(\b[21] ), .c(new_n224), .o1(new_n240));
  tech160nm_fiao0012aa1n02p5x5 g145(.a(new_n240), .b(new_n219), .c(new_n239), .o(new_n241));
  norb02aa1n06x5               g146(.a(new_n233), .b(new_n241), .out0(new_n242));
  tech160nm_finand02aa1n03p5x5 g147(.a(new_n234), .b(new_n242), .o1(new_n243));
  aoi013aa1n02x4               g148(.a(new_n241), .b(new_n183), .c(new_n214), .d(new_n239), .o1(new_n244));
  oai012aa1n02x5               g149(.a(new_n243), .b(new_n244), .c(new_n233), .o1(\s[23] ));
  and002aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o(new_n246));
  xorc02aa1n02x5               g151(.a(\a[24] ), .b(\b[23] ), .out0(new_n247));
  aoai13aa1n04x5               g152(.a(new_n247), .b(new_n246), .c(new_n234), .d(new_n242), .o1(new_n248));
  nona22aa1n03x5               g153(.a(new_n243), .b(new_n247), .c(new_n246), .out0(new_n249));
  nanp02aa1n03x5               g154(.a(new_n249), .b(new_n248), .o1(\s[24] ));
  inv040aa1d30x5               g155(.a(\a[23] ), .o1(new_n251));
  inv040aa1d30x5               g156(.a(\b[22] ), .o1(new_n252));
  nand42aa1n03x5               g157(.a(new_n252), .b(new_n251), .o1(new_n253));
  nand42aa1n03x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  orn002aa1n24x5               g159(.a(\a[24] ), .b(\b[23] ), .o(new_n255));
  oai112aa1n06x5               g160(.a(new_n255), .b(new_n254), .c(new_n252), .d(new_n251), .o1(new_n256));
  nano23aa1n06x5               g161(.a(new_n256), .b(new_n225), .c(new_n228), .d(new_n253), .out0(new_n257));
  inv000aa1n06x5               g162(.a(new_n257), .o1(new_n258));
  nano32aa1n02x4               g163(.a(new_n258), .b(new_n194), .c(new_n206), .d(new_n208), .out0(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  inv020aa1n04x5               g165(.a(new_n256), .o1(new_n261));
  oaoi03aa1n02x5               g166(.a(\a[24] ), .b(\b[23] ), .c(new_n253), .o1(new_n262));
  aoi013aa1n06x4               g167(.a(new_n262), .b(new_n261), .c(new_n240), .d(new_n253), .o1(new_n263));
  nano32aa1n03x7               g168(.a(new_n225), .b(new_n253), .c(new_n237), .d(new_n238), .out0(new_n264));
  nona23aa1n09x5               g169(.a(new_n218), .b(new_n264), .c(new_n256), .d(new_n216), .out0(new_n265));
  nand22aa1n09x5               g170(.a(new_n265), .b(new_n263), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  aoai13aa1n04x5               g172(.a(new_n267), .b(new_n260), .c(new_n193), .d(new_n180), .o1(new_n268));
  xorb03aa1n02x5               g173(.a(new_n268), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n06x5               g174(.a(\b[24] ), .b(\a[25] ), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[25] ), .b(\b[24] ), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n266), .c(new_n183), .d(new_n259), .o1(new_n273));
  xorc02aa1n12x5               g178(.a(\a[26] ), .b(\b[25] ), .out0(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  tech160nm_fiaoi012aa1n03p5x5 g180(.a(new_n275), .b(new_n273), .c(new_n271), .o1(new_n276));
  aoi112aa1n02x5               g181(.a(new_n270), .b(new_n274), .c(new_n268), .d(new_n272), .o1(new_n277));
  nor002aa1n02x5               g182(.a(new_n276), .b(new_n277), .o1(\s[26] ));
  and002aa1n06x5               g183(.a(new_n274), .b(new_n272), .o(new_n279));
  nand23aa1n06x5               g184(.a(new_n214), .b(new_n257), .c(new_n279), .o1(new_n280));
  oao003aa1n02x5               g185(.a(\a[26] ), .b(\b[25] ), .c(new_n271), .carry(new_n281));
  inv000aa1d42x5               g186(.a(new_n281), .o1(new_n282));
  aoi012aa1n06x5               g187(.a(new_n282), .b(new_n266), .c(new_n279), .o1(new_n283));
  aoai13aa1n04x5               g188(.a(new_n283), .b(new_n280), .c(new_n193), .d(new_n180), .o1(new_n284));
  xorb03aa1n03x5               g189(.a(new_n284), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1n03x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  inv000aa1n06x5               g191(.a(new_n286), .o1(new_n287));
  xorc02aa1n12x5               g192(.a(\a[28] ), .b(\b[27] ), .out0(new_n288));
  inv000aa1d42x5               g193(.a(new_n288), .o1(new_n289));
  nanp02aa1n02x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  inv000aa1n02x5               g195(.a(new_n280), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n279), .o1(new_n292));
  aoai13aa1n06x5               g197(.a(new_n281), .b(new_n292), .c(new_n265), .d(new_n263), .o1(new_n293));
  aoai13aa1n03x5               g198(.a(new_n290), .b(new_n293), .c(new_n183), .d(new_n291), .o1(new_n294));
  tech160nm_fiaoi012aa1n03p5x5 g199(.a(new_n289), .b(new_n294), .c(new_n287), .o1(new_n295));
  aoi112aa1n03x4               g200(.a(new_n288), .b(new_n286), .c(new_n284), .d(new_n290), .o1(new_n296));
  nor002aa1n02x5               g201(.a(new_n295), .b(new_n296), .o1(\s[28] ));
  nano22aa1n02x4               g202(.a(new_n289), .b(new_n287), .c(new_n290), .out0(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n293), .c(new_n183), .d(new_n291), .o1(new_n299));
  oao003aa1n12x5               g204(.a(\a[28] ), .b(\b[27] ), .c(new_n287), .carry(new_n300));
  xorc02aa1n12x5               g205(.a(\a[29] ), .b(\b[28] ), .out0(new_n301));
  inv000aa1d42x5               g206(.a(new_n301), .o1(new_n302));
  tech160nm_fiaoi012aa1n03p5x5 g207(.a(new_n302), .b(new_n299), .c(new_n300), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n300), .o1(new_n304));
  aoi112aa1n03x4               g209(.a(new_n301), .b(new_n304), .c(new_n284), .d(new_n298), .o1(new_n305));
  nor002aa1n02x5               g210(.a(new_n303), .b(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n118), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano32aa1n02x4               g212(.a(new_n302), .b(new_n288), .c(new_n290), .d(new_n287), .out0(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n293), .c(new_n183), .d(new_n291), .o1(new_n309));
  oao003aa1n12x5               g214(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .carry(new_n310));
  xorc02aa1n12x5               g215(.a(\a[30] ), .b(\b[29] ), .out0(new_n311));
  inv000aa1d42x5               g216(.a(new_n311), .o1(new_n312));
  tech160nm_fiaoi012aa1n03p5x5 g217(.a(new_n312), .b(new_n309), .c(new_n310), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n310), .o1(new_n314));
  aoi112aa1n03x4               g219(.a(new_n311), .b(new_n314), .c(new_n284), .d(new_n308), .o1(new_n315));
  nor002aa1n02x5               g220(.a(new_n313), .b(new_n315), .o1(\s[30] ));
  xnrc02aa1n02x5               g221(.a(\b[30] ), .b(\a[31] ), .out0(new_n317));
  and003aa1n02x5               g222(.a(new_n298), .b(new_n311), .c(new_n301), .o(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n293), .c(new_n183), .d(new_n291), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[30] ), .b(\b[29] ), .c(new_n310), .carry(new_n320));
  tech160nm_fiaoi012aa1n02p5x5 g225(.a(new_n317), .b(new_n319), .c(new_n320), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n317), .o1(new_n322));
  inv000aa1n02x5               g227(.a(new_n320), .o1(new_n323));
  aoi112aa1n03x4               g228(.a(new_n322), .b(new_n323), .c(new_n284), .d(new_n318), .o1(new_n324));
  norp02aa1n03x5               g229(.a(new_n321), .b(new_n324), .o1(\s[31] ));
  xnrb03aa1n02x5               g230(.a(new_n120), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g231(.a(\a[3] ), .b(\b[2] ), .c(new_n120), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g233(.a(new_n123), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g234(.a(new_n125), .b(new_n123), .c(new_n108), .o1(new_n330));
  xnrb03aa1n02x5               g235(.a(new_n330), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oab012aa1n03x5               g236(.a(new_n107), .b(new_n330), .c(new_n124), .out0(new_n332));
  xnbna2aa1n03x5               g237(.a(new_n332), .b(new_n110), .c(new_n104), .out0(\s[7] ));
  oaoi03aa1n02x5               g238(.a(\a[7] ), .b(\b[6] ), .c(new_n332), .o1(new_n334));
  xorb03aa1n02x5               g239(.a(new_n334), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g240(.a(new_n146), .b(new_n128), .c(new_n101), .out0(\s[9] ));
endmodule


