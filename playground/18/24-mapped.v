// Benchmark "adder" written by ABC on Wed Jul 17 21:23:13 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n332, new_n335, new_n337, new_n339;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n12x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nand02aa1n03x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nona23aa1n09x5               g005(.a(new_n100), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n101));
  nor002aa1n02x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  nanb02aa1n02x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  xnrc02aa1n02x5               g009(.a(\b[4] ), .b(\a[5] ), .out0(new_n105));
  nor043aa1n03x5               g010(.a(new_n101), .b(new_n104), .c(new_n105), .o1(new_n106));
  nor042aa1n02x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  norp02aa1n03x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nona23aa1n03x5               g015(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n111));
  tech160nm_finand02aa1n03p5x5 g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  nand22aa1n12x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  nor042aa1n06x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  oai012aa1n12x5               g019(.a(new_n112), .b(new_n114), .c(new_n113), .o1(new_n115));
  oai012aa1n02x5               g020(.a(new_n108), .b(new_n109), .c(new_n107), .o1(new_n116));
  tech160nm_fioai012aa1n05x5   g021(.a(new_n116), .b(new_n111), .c(new_n115), .o1(new_n117));
  inv000aa1d42x5               g022(.a(new_n97), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(new_n99), .b(new_n98), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\a[5] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[4] ), .o1(new_n121));
  aoai13aa1n02x5               g026(.a(new_n103), .b(new_n102), .c(new_n120), .d(new_n121), .o1(new_n122));
  oai112aa1n04x5               g027(.a(new_n118), .b(new_n119), .c(new_n101), .d(new_n122), .o1(new_n123));
  aoi012aa1n02x5               g028(.a(new_n123), .b(new_n117), .c(new_n106), .o1(new_n124));
  oaoi03aa1n02x5               g029(.a(\a[9] ), .b(\b[8] ), .c(new_n124), .o1(new_n125));
  xorb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand42aa1n03x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nand42aa1n03x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nona23aa1n02x4               g035(.a(new_n130), .b(new_n128), .c(new_n127), .d(new_n129), .out0(new_n131));
  tech160nm_fioai012aa1n04x5   g036(.a(new_n128), .b(new_n129), .c(new_n127), .o1(new_n132));
  oai012aa1n02x5               g037(.a(new_n132), .b(new_n124), .c(new_n131), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  aoi012aa1n02x5               g041(.a(new_n135), .b(new_n133), .c(new_n136), .o1(new_n137));
  nor002aa1n12x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n138), .o1(new_n139));
  nand42aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n137), .b(new_n140), .c(new_n139), .out0(\s[12] ));
  nano23aa1n02x4               g046(.a(new_n97), .b(new_n99), .c(new_n100), .d(new_n98), .out0(new_n142));
  nona22aa1n02x4               g047(.a(new_n142), .b(new_n105), .c(new_n104), .out0(new_n143));
  nano23aa1n02x4               g048(.a(new_n107), .b(new_n109), .c(new_n110), .d(new_n108), .out0(new_n144));
  inv000aa1d42x5               g049(.a(new_n115), .o1(new_n145));
  aobi12aa1n03x5               g050(.a(new_n116), .b(new_n144), .c(new_n145), .out0(new_n146));
  norp02aa1n02x5               g051(.a(new_n101), .b(new_n122), .o1(new_n147));
  nano22aa1n02x4               g052(.a(new_n147), .b(new_n118), .c(new_n119), .out0(new_n148));
  tech160nm_fioai012aa1n05x5   g053(.a(new_n148), .b(new_n146), .c(new_n143), .o1(new_n149));
  nano23aa1n02x5               g054(.a(new_n127), .b(new_n129), .c(new_n130), .d(new_n128), .out0(new_n150));
  nano23aa1n06x5               g055(.a(new_n135), .b(new_n138), .c(new_n140), .d(new_n136), .out0(new_n151));
  nanp02aa1n02x5               g056(.a(new_n135), .b(new_n140), .o1(new_n152));
  nona23aa1d18x5               g057(.a(new_n140), .b(new_n136), .c(new_n135), .d(new_n138), .out0(new_n153));
  oai112aa1n06x5               g058(.a(new_n152), .b(new_n139), .c(new_n153), .d(new_n132), .o1(new_n154));
  aoi013aa1n02x4               g059(.a(new_n154), .b(new_n149), .c(new_n150), .d(new_n151), .o1(new_n155));
  nor042aa1n04x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  inv000aa1n02x5               g061(.a(new_n156), .o1(new_n157));
  nand42aa1n03x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n155), .b(new_n158), .c(new_n157), .out0(\s[13] ));
  oaoi03aa1n03x5               g064(.a(\a[13] ), .b(\b[12] ), .c(new_n155), .o1(new_n160));
  nor042aa1n03x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n03x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanb02aa1n02x5               g067(.a(new_n161), .b(new_n162), .out0(new_n163));
  xnrc02aa1n03x5               g068(.a(new_n160), .b(new_n163), .out0(\s[14] ));
  nano23aa1n03x7               g069(.a(new_n156), .b(new_n161), .c(new_n162), .d(new_n158), .out0(new_n165));
  oaoi03aa1n02x5               g070(.a(\a[14] ), .b(\b[13] ), .c(new_n157), .o1(new_n166));
  aoi012aa1n02x5               g071(.a(new_n166), .b(new_n154), .c(new_n165), .o1(new_n167));
  nona23aa1n02x4               g072(.a(new_n162), .b(new_n158), .c(new_n156), .d(new_n161), .out0(new_n168));
  nona32aa1n02x4               g073(.a(new_n149), .b(new_n168), .c(new_n153), .d(new_n131), .out0(new_n169));
  nor042aa1n09x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  tech160nm_finand02aa1n03p5x5 g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  xobna2aa1n03x5               g077(.a(new_n172), .b(new_n169), .c(new_n167), .out0(\s[15] ));
  inv000aa1d42x5               g078(.a(new_n170), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n172), .c(new_n169), .d(new_n167), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  nor042aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand42aa1n04x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nano23aa1n09x5               g083(.a(new_n170), .b(new_n177), .c(new_n178), .d(new_n171), .out0(new_n179));
  aoi122aa1n06x5               g084(.a(new_n177), .b(new_n178), .c(new_n170), .d(new_n179), .e(new_n166), .o1(new_n180));
  nona23aa1n02x4               g085(.a(new_n178), .b(new_n171), .c(new_n170), .d(new_n177), .out0(new_n181));
  nona22aa1n06x5               g086(.a(new_n154), .b(new_n168), .c(new_n181), .out0(new_n182));
  nanp02aa1n02x5               g087(.a(new_n179), .b(new_n150), .o1(new_n183));
  nano22aa1n03x7               g088(.a(new_n183), .b(new_n151), .c(new_n165), .out0(new_n184));
  aoai13aa1n12x5               g089(.a(new_n184), .b(new_n123), .c(new_n117), .d(new_n106), .o1(new_n185));
  nand23aa1n09x5               g090(.a(new_n185), .b(new_n180), .c(new_n182), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g092(.a(\a[18] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\a[17] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\b[16] ), .o1(new_n190));
  oaoi03aa1n03x5               g095(.a(new_n189), .b(new_n190), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n188), .out0(\s[18] ));
  inv020aa1n03x5               g097(.a(new_n180), .o1(new_n193));
  aoi013aa1n06x4               g098(.a(new_n193), .b(new_n154), .c(new_n165), .d(new_n179), .o1(new_n194));
  xroi22aa1d06x4               g099(.a(new_n189), .b(\b[16] ), .c(new_n188), .d(\b[17] ), .out0(new_n195));
  inv000aa1d42x5               g100(.a(new_n195), .o1(new_n196));
  oai022aa1n09x5               g101(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n197));
  oaib12aa1n18x5               g102(.a(new_n197), .b(new_n188), .c(\b[17] ), .out0(new_n198));
  aoai13aa1n03x5               g103(.a(new_n198), .b(new_n196), .c(new_n194), .d(new_n185), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanp02aa1n04x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanb02aa1n06x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  inv000aa1d42x5               g110(.a(\b[19] ), .o1(new_n206));
  nanb02aa1n06x5               g111(.a(\a[20] ), .b(new_n206), .out0(new_n207));
  nand02aa1n04x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nand02aa1n06x5               g113(.a(new_n207), .b(new_n208), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  aoi112aa1n03x4               g115(.a(new_n202), .b(new_n210), .c(new_n199), .d(new_n205), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n202), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n198), .o1(new_n213));
  aoai13aa1n03x5               g118(.a(new_n205), .b(new_n213), .c(new_n186), .d(new_n195), .o1(new_n214));
  tech160nm_fiaoi012aa1n05x5   g119(.a(new_n209), .b(new_n214), .c(new_n212), .o1(new_n215));
  norp02aa1n03x5               g120(.a(new_n215), .b(new_n211), .o1(\s[20] ));
  norp02aa1n04x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nona23aa1d18x5               g122(.a(new_n208), .b(new_n203), .c(new_n202), .d(new_n217), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  nand22aa1n12x5               g124(.a(new_n195), .b(new_n219), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(new_n202), .b(new_n208), .o1(new_n221));
  nor043aa1n03x5               g126(.a(new_n198), .b(new_n204), .c(new_n209), .o1(new_n222));
  nano22aa1n03x7               g127(.a(new_n222), .b(new_n207), .c(new_n221), .out0(new_n223));
  aoai13aa1n04x5               g128(.a(new_n223), .b(new_n220), .c(new_n194), .d(new_n185), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  tech160nm_fixorc02aa1n04x5   g131(.a(\a[21] ), .b(\b[20] ), .out0(new_n227));
  xorc02aa1n12x5               g132(.a(\a[22] ), .b(\b[21] ), .out0(new_n228));
  aoi112aa1n02x7               g133(.a(new_n226), .b(new_n228), .c(new_n224), .d(new_n227), .o1(new_n229));
  inv000aa1n02x5               g134(.a(new_n226), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n220), .o1(new_n231));
  oai112aa1n06x5               g136(.a(new_n221), .b(new_n207), .c(new_n218), .d(new_n198), .o1(new_n232));
  aoai13aa1n06x5               g137(.a(new_n227), .b(new_n232), .c(new_n186), .d(new_n231), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n228), .o1(new_n234));
  tech160nm_fiaoi012aa1n05x5   g139(.a(new_n234), .b(new_n233), .c(new_n230), .o1(new_n235));
  norp02aa1n02x5               g140(.a(new_n235), .b(new_n229), .o1(\s[22] ));
  and002aa1n06x5               g141(.a(new_n228), .b(new_n227), .o(new_n237));
  oaoi03aa1n09x5               g142(.a(\a[22] ), .b(\b[21] ), .c(new_n230), .o1(new_n238));
  aoi012aa1d18x5               g143(.a(new_n238), .b(new_n232), .c(new_n237), .o1(new_n239));
  nano22aa1n03x7               g144(.a(new_n220), .b(new_n227), .c(new_n228), .out0(new_n240));
  inv020aa1n03x5               g145(.a(new_n240), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n239), .b(new_n241), .c(new_n194), .d(new_n185), .o1(new_n242));
  xorb03aa1n02x5               g147(.a(new_n242), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n04x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  nanp02aa1n02x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n245), .b(new_n244), .out0(new_n246));
  nor042aa1n02x5               g151(.a(\b[23] ), .b(\a[24] ), .o1(new_n247));
  nanp02aa1n02x5               g152(.a(\b[23] ), .b(\a[24] ), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n248), .b(new_n247), .out0(new_n249));
  aoi112aa1n03x5               g154(.a(new_n244), .b(new_n249), .c(new_n242), .d(new_n246), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n244), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n239), .o1(new_n252));
  aoai13aa1n03x5               g157(.a(new_n246), .b(new_n252), .c(new_n186), .d(new_n240), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n249), .o1(new_n254));
  tech160nm_fiaoi012aa1n03p5x5 g159(.a(new_n254), .b(new_n253), .c(new_n251), .o1(new_n255));
  nor042aa1n03x5               g160(.a(new_n255), .b(new_n250), .o1(\s[24] ));
  nano23aa1n06x5               g161(.a(new_n244), .b(new_n247), .c(new_n248), .d(new_n245), .out0(new_n257));
  nano32aa1n03x7               g162(.a(new_n220), .b(new_n257), .c(new_n227), .d(new_n228), .out0(new_n258));
  inv020aa1n03x5               g163(.a(new_n258), .o1(new_n259));
  aoi112aa1n02x5               g164(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n260));
  nand42aa1n02x5               g165(.a(new_n257), .b(new_n238), .o1(new_n261));
  nona22aa1n03x5               g166(.a(new_n261), .b(new_n260), .c(new_n247), .out0(new_n262));
  nanp03aa1n03x5               g167(.a(new_n257), .b(new_n227), .c(new_n228), .o1(new_n263));
  inv040aa1n03x5               g168(.a(new_n263), .o1(new_n264));
  aoi012aa1d18x5               g169(.a(new_n262), .b(new_n232), .c(new_n264), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n259), .c(new_n194), .d(new_n185), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  xorc02aa1n02x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  xorc02aa1n12x5               g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  aoi112aa1n03x5               g175(.a(new_n268), .b(new_n270), .c(new_n266), .d(new_n269), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n268), .o1(new_n272));
  inv000aa1n02x5               g177(.a(new_n265), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n269), .b(new_n273), .c(new_n186), .d(new_n258), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n270), .o1(new_n275));
  tech160nm_fiaoi012aa1n03p5x5 g180(.a(new_n275), .b(new_n274), .c(new_n272), .o1(new_n276));
  nor042aa1n03x5               g181(.a(new_n276), .b(new_n271), .o1(\s[26] ));
  nanp02aa1n03x5               g182(.a(new_n182), .b(new_n180), .o1(new_n278));
  nor002aa1n02x5               g183(.a(new_n181), .b(new_n131), .o1(new_n279));
  nona22aa1n02x4               g184(.a(new_n279), .b(new_n168), .c(new_n153), .out0(new_n280));
  oaoi13aa1n09x5               g185(.a(new_n280), .b(new_n148), .c(new_n146), .d(new_n143), .o1(new_n281));
  and002aa1n02x5               g186(.a(new_n270), .b(new_n269), .o(new_n282));
  nano32aa1d12x5               g187(.a(new_n220), .b(new_n282), .c(new_n237), .d(new_n257), .out0(new_n283));
  oai012aa1n12x5               g188(.a(new_n283), .b(new_n278), .c(new_n281), .o1(new_n284));
  norp02aa1n02x5               g189(.a(\b[25] ), .b(\a[26] ), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  aoi112aa1n02x5               g191(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n287));
  inv000aa1n02x5               g192(.a(new_n287), .o1(new_n288));
  aoi112aa1n02x5               g193(.a(new_n260), .b(new_n247), .c(new_n257), .d(new_n238), .o1(new_n289));
  inv000aa1n02x5               g194(.a(new_n282), .o1(new_n290));
  oaoi13aa1n03x5               g195(.a(new_n290), .b(new_n289), .c(new_n223), .d(new_n263), .o1(new_n291));
  nano22aa1n03x7               g196(.a(new_n291), .b(new_n286), .c(new_n288), .out0(new_n292));
  nor042aa1n03x5               g197(.a(\b[26] ), .b(\a[27] ), .o1(new_n293));
  nanp02aa1n02x5               g198(.a(\b[26] ), .b(\a[27] ), .o1(new_n294));
  norb02aa1n02x5               g199(.a(new_n294), .b(new_n293), .out0(new_n295));
  xnbna2aa1n03x5               g200(.a(new_n295), .b(new_n284), .c(new_n292), .out0(\s[27] ));
  xorc02aa1n02x5               g201(.a(\a[28] ), .b(\b[27] ), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n282), .b(new_n262), .c(new_n232), .d(new_n264), .o1(new_n298));
  nona22aa1n03x5               g203(.a(new_n298), .b(new_n287), .c(new_n285), .out0(new_n299));
  aoi112aa1n03x4               g204(.a(new_n299), .b(new_n293), .c(new_n186), .d(new_n283), .o1(new_n300));
  nano22aa1n03x5               g205(.a(new_n300), .b(new_n294), .c(new_n297), .out0(new_n301));
  inv000aa1d42x5               g206(.a(new_n293), .o1(new_n302));
  nanp03aa1n03x5               g207(.a(new_n284), .b(new_n292), .c(new_n302), .o1(new_n303));
  aoi012aa1n03x5               g208(.a(new_n297), .b(new_n303), .c(new_n294), .o1(new_n304));
  nor002aa1n02x5               g209(.a(new_n304), .b(new_n301), .o1(\s[28] ));
  and002aa1n02x5               g210(.a(new_n297), .b(new_n295), .o(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n299), .c(new_n186), .d(new_n283), .o1(new_n307));
  oao003aa1n02x5               g212(.a(\a[28] ), .b(\b[27] ), .c(new_n302), .carry(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[28] ), .b(\a[29] ), .out0(new_n309));
  tech160nm_fiaoi012aa1n02p5x5 g214(.a(new_n309), .b(new_n307), .c(new_n308), .o1(new_n310));
  aobi12aa1n02x7               g215(.a(new_n306), .b(new_n284), .c(new_n292), .out0(new_n311));
  nano22aa1n03x5               g216(.a(new_n311), .b(new_n308), .c(new_n309), .out0(new_n312));
  norp02aa1n03x5               g217(.a(new_n310), .b(new_n312), .o1(\s[29] ));
  xorb03aa1n02x5               g218(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g219(.a(new_n309), .b(new_n297), .c(new_n295), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n299), .c(new_n186), .d(new_n283), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[29] ), .b(\b[28] ), .c(new_n308), .carry(new_n317));
  xnrc02aa1n02x5               g222(.a(\b[29] ), .b(\a[30] ), .out0(new_n318));
  tech160nm_fiaoi012aa1n02p5x5 g223(.a(new_n318), .b(new_n316), .c(new_n317), .o1(new_n319));
  aobi12aa1n02x7               g224(.a(new_n315), .b(new_n284), .c(new_n292), .out0(new_n320));
  nano22aa1n03x5               g225(.a(new_n320), .b(new_n317), .c(new_n318), .out0(new_n321));
  norp02aa1n03x5               g226(.a(new_n319), .b(new_n321), .o1(\s[30] ));
  nano23aa1n02x4               g227(.a(new_n318), .b(new_n309), .c(new_n297), .d(new_n295), .out0(new_n323));
  aobi12aa1n02x7               g228(.a(new_n323), .b(new_n284), .c(new_n292), .out0(new_n324));
  oao003aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .c(new_n317), .carry(new_n325));
  xnrc02aa1n02x5               g230(.a(\b[30] ), .b(\a[31] ), .out0(new_n326));
  nano22aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n326), .out0(new_n327));
  aoai13aa1n03x5               g232(.a(new_n323), .b(new_n299), .c(new_n186), .d(new_n283), .o1(new_n328));
  tech160nm_fiaoi012aa1n02p5x5 g233(.a(new_n326), .b(new_n328), .c(new_n325), .o1(new_n329));
  norp02aa1n03x5               g234(.a(new_n329), .b(new_n327), .o1(\s[31] ));
  xnrb03aa1n02x5               g235(.a(new_n115), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g236(.a(\a[3] ), .b(\b[2] ), .c(new_n115), .o1(new_n332));
  xorb03aa1n02x5               g237(.a(new_n332), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g238(.a(new_n117), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g239(.a(new_n120), .b(new_n121), .c(new_n117), .o1(new_n335));
  xnrb03aa1n02x5               g240(.a(new_n335), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g241(.a(\a[6] ), .b(\b[5] ), .c(new_n335), .o1(new_n337));
  xorb03aa1n02x5               g242(.a(new_n337), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g243(.a(new_n99), .b(new_n337), .c(new_n100), .o1(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n339), .b(new_n118), .c(new_n98), .out0(\s[8] ));
  xorb03aa1n02x5               g245(.a(new_n149), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


