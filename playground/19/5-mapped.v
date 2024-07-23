// Benchmark "adder" written by ABC on Wed Jul 17 21:42:15 2024

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
    new_n133, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n321, new_n323, new_n324, new_n326;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  nor002aa1n03x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  and002aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o(new_n99));
  inv000aa1d42x5               g004(.a(new_n99), .o1(new_n100));
  nor002aa1n20x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nand02aa1n08x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nor002aa1d32x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nona23aa1d18x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  tech160nm_fixnrc02aa1n04x5   g010(.a(\b[5] ), .b(\a[6] ), .out0(new_n106));
  tech160nm_fixnrc02aa1n05x5   g011(.a(\b[4] ), .b(\a[5] ), .out0(new_n107));
  nor043aa1d12x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o1(new_n108));
  nor022aa1n06x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  nand02aa1n06x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nor022aa1n12x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nand42aa1n02x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nona23aa1n09x5               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  inv000aa1d42x5               g018(.a(\a[2] ), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\b[1] ), .o1(new_n115));
  nand02aa1d04x5               g020(.a(\b[0] ), .b(\a[1] ), .o1(new_n116));
  oaoi03aa1n09x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  tech160nm_fioai012aa1n03p5x5 g022(.a(new_n110), .b(new_n111), .c(new_n109), .o1(new_n118));
  oai012aa1d24x5               g023(.a(new_n118), .b(new_n113), .c(new_n117), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n103), .b(new_n102), .o1(new_n120));
  oaih22aa1d12x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  aob012aa1d18x5               g026(.a(new_n121), .b(\b[5] ), .c(\a[6] ), .out0(new_n122));
  oai122aa1n12x5               g027(.a(new_n120), .b(new_n105), .c(new_n122), .d(\b[7] ), .e(\a[8] ), .o1(new_n123));
  aoi012aa1d18x5               g028(.a(new_n123), .b(new_n108), .c(new_n119), .o1(new_n124));
  inv000aa1d42x5               g029(.a(new_n124), .o1(new_n125));
  aoai13aa1n02x5               g030(.a(new_n97), .b(new_n98), .c(new_n125), .d(new_n100), .o1(new_n126));
  aoai13aa1n02x5               g031(.a(new_n100), .b(new_n123), .c(new_n108), .d(new_n119), .o1(new_n127));
  nona22aa1n02x4               g032(.a(new_n127), .b(new_n98), .c(new_n97), .out0(new_n128));
  nanp02aa1n02x5               g033(.a(new_n126), .b(new_n128), .o1(\s[10] ));
  nanp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nor042aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand22aa1n09x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n128), .c(new_n130), .out0(\s[11] ));
  aoi013aa1n02x4               g039(.a(new_n131), .b(new_n128), .c(new_n130), .d(new_n132), .o1(new_n135));
  xnrb03aa1n02x5               g040(.a(new_n135), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor022aa1n06x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand22aa1n09x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nona23aa1n09x5               g043(.a(new_n138), .b(new_n132), .c(new_n131), .d(new_n137), .out0(new_n139));
  inv000aa1d42x5               g044(.a(\a[10] ), .o1(new_n140));
  inv000aa1d42x5               g045(.a(\b[9] ), .o1(new_n141));
  oaoi03aa1n02x5               g046(.a(new_n140), .b(new_n141), .c(new_n98), .o1(new_n142));
  tech160nm_fiao0012aa1n02p5x5 g047(.a(new_n137), .b(new_n131), .c(new_n138), .o(new_n143));
  oabi12aa1n06x5               g048(.a(new_n143), .b(new_n139), .c(new_n142), .out0(new_n144));
  xnrc02aa1n12x5               g049(.a(\b[8] ), .b(\a[9] ), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  norp02aa1n02x5               g051(.a(new_n139), .b(new_n97), .o1(new_n147));
  nano22aa1n12x5               g052(.a(new_n124), .b(new_n146), .c(new_n147), .out0(new_n148));
  norp02aa1n02x5               g053(.a(new_n148), .b(new_n144), .o1(new_n149));
  nor002aa1d32x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  nand02aa1n06x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n149), .b(new_n152), .c(new_n151), .out0(\s[13] ));
  oaoi13aa1n06x5               g058(.a(new_n150), .b(new_n152), .c(new_n148), .d(new_n144), .o1(new_n154));
  xnrb03aa1n03x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n16x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nand02aa1d06x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nona23aa1n09x5               g062(.a(new_n157), .b(new_n152), .c(new_n150), .d(new_n156), .out0(new_n158));
  inv040aa1n04x5               g063(.a(new_n158), .o1(new_n159));
  nano32aa1n03x7               g064(.a(new_n124), .b(new_n159), .c(new_n146), .d(new_n147), .out0(new_n160));
  nano23aa1n06x5               g065(.a(new_n131), .b(new_n137), .c(new_n138), .d(new_n132), .out0(new_n161));
  oao003aa1n03x5               g066(.a(new_n140), .b(new_n141), .c(new_n98), .carry(new_n162));
  aoai13aa1n06x5               g067(.a(new_n159), .b(new_n143), .c(new_n161), .d(new_n162), .o1(new_n163));
  aoi012aa1d18x5               g068(.a(new_n156), .b(new_n150), .c(new_n157), .o1(new_n164));
  nano22aa1n03x7               g069(.a(new_n160), .b(new_n163), .c(new_n164), .out0(new_n165));
  xnrb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  oaoi03aa1n03x5               g071(.a(\a[15] ), .b(\b[14] ), .c(new_n165), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  norp02aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nand42aa1n03x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nor022aa1n06x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nand22aa1n04x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nano23aa1n03x7               g077(.a(new_n169), .b(new_n171), .c(new_n172), .d(new_n170), .out0(new_n173));
  nona22aa1n02x4               g078(.a(new_n161), .b(new_n145), .c(new_n97), .out0(new_n174));
  nano22aa1n03x7               g079(.a(new_n174), .b(new_n159), .c(new_n173), .out0(new_n175));
  aoai13aa1n06x5               g080(.a(new_n175), .b(new_n123), .c(new_n108), .d(new_n119), .o1(new_n176));
  inv000aa1d42x5               g081(.a(new_n164), .o1(new_n177));
  aoai13aa1n04x5               g082(.a(new_n173), .b(new_n177), .c(new_n144), .d(new_n159), .o1(new_n178));
  aoi012aa1n02x7               g083(.a(new_n171), .b(new_n169), .c(new_n172), .o1(new_n179));
  nand23aa1n06x5               g084(.a(new_n176), .b(new_n178), .c(new_n179), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g086(.a(\a[18] ), .o1(new_n182));
  inv040aa1d32x5               g087(.a(\a[17] ), .o1(new_n183));
  inv040aa1n12x5               g088(.a(\b[16] ), .o1(new_n184));
  oaoi03aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n180), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[17] ), .c(new_n182), .out0(\s[18] ));
  nand22aa1n12x5               g091(.a(new_n119), .b(new_n108), .o1(new_n187));
  nor042aa1n02x5               g092(.a(new_n105), .b(new_n122), .o1(new_n188));
  aoi112aa1n06x5               g093(.a(new_n188), .b(new_n101), .c(new_n102), .d(new_n103), .o1(new_n189));
  nona23aa1n09x5               g094(.a(new_n172), .b(new_n170), .c(new_n169), .d(new_n171), .out0(new_n190));
  nor022aa1n02x5               g095(.a(new_n190), .b(new_n158), .o1(new_n191));
  nona32aa1n03x5               g096(.a(new_n191), .b(new_n139), .c(new_n145), .d(new_n97), .out0(new_n192));
  aoi012aa1d24x5               g097(.a(new_n192), .b(new_n187), .c(new_n189), .o1(new_n193));
  aoai13aa1n12x5               g098(.a(new_n179), .b(new_n190), .c(new_n163), .d(new_n164), .o1(new_n194));
  xroi22aa1d06x4               g099(.a(new_n183), .b(\b[16] ), .c(new_n182), .d(\b[17] ), .out0(new_n195));
  oai012aa1n06x5               g100(.a(new_n195), .b(new_n194), .c(new_n193), .o1(new_n196));
  oaih22aa1n06x5               g101(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n197));
  oaib12aa1n09x5               g102(.a(new_n197), .b(new_n182), .c(\b[17] ), .out0(new_n198));
  nor002aa1d32x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand42aa1d28x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  xnbna2aa1n03x5               g107(.a(new_n202), .b(new_n196), .c(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g109(.a(new_n199), .o1(new_n205));
  tech160nm_fiaoi012aa1n05x5   g110(.a(new_n201), .b(new_n196), .c(new_n198), .o1(new_n206));
  norp02aa1n24x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nand42aa1d28x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(new_n209));
  nano22aa1n03x7               g114(.a(new_n206), .b(new_n205), .c(new_n209), .out0(new_n210));
  nand22aa1n02x5               g115(.a(new_n184), .b(new_n183), .o1(new_n211));
  oaoi03aa1n09x5               g116(.a(\a[18] ), .b(\b[17] ), .c(new_n211), .o1(new_n212));
  aoai13aa1n03x5               g117(.a(new_n202), .b(new_n212), .c(new_n180), .d(new_n195), .o1(new_n213));
  tech160nm_fiaoi012aa1n02p5x5 g118(.a(new_n209), .b(new_n213), .c(new_n205), .o1(new_n214));
  nor002aa1n02x5               g119(.a(new_n214), .b(new_n210), .o1(\s[20] ));
  nano23aa1n09x5               g120(.a(new_n199), .b(new_n207), .c(new_n208), .d(new_n200), .out0(new_n216));
  nand02aa1d04x5               g121(.a(new_n195), .b(new_n216), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  oai012aa1n06x5               g123(.a(new_n218), .b(new_n194), .c(new_n193), .o1(new_n219));
  nona23aa1n09x5               g124(.a(new_n208), .b(new_n200), .c(new_n199), .d(new_n207), .out0(new_n220));
  aoi012aa1n12x5               g125(.a(new_n207), .b(new_n199), .c(new_n208), .o1(new_n221));
  oai012aa1n18x5               g126(.a(new_n221), .b(new_n220), .c(new_n198), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  nor042aa1d18x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  nand42aa1n02x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  xnbna2aa1n03x5               g131(.a(new_n226), .b(new_n219), .c(new_n223), .out0(\s[21] ));
  inv030aa1n06x5               g132(.a(new_n224), .o1(new_n228));
  aobi12aa1n06x5               g133(.a(new_n226), .b(new_n219), .c(new_n223), .out0(new_n229));
  xnrc02aa1n12x5               g134(.a(\b[21] ), .b(\a[22] ), .out0(new_n230));
  nano22aa1n03x7               g135(.a(new_n229), .b(new_n228), .c(new_n230), .out0(new_n231));
  aoai13aa1n03x5               g136(.a(new_n226), .b(new_n222), .c(new_n180), .d(new_n218), .o1(new_n232));
  aoi012aa1n03x5               g137(.a(new_n230), .b(new_n232), .c(new_n228), .o1(new_n233));
  norp02aa1n03x5               g138(.a(new_n233), .b(new_n231), .o1(\s[22] ));
  nano22aa1n12x5               g139(.a(new_n230), .b(new_n228), .c(new_n225), .out0(new_n235));
  and003aa1n02x5               g140(.a(new_n195), .b(new_n235), .c(new_n216), .o(new_n236));
  oai012aa1n06x5               g141(.a(new_n236), .b(new_n194), .c(new_n193), .o1(new_n237));
  oao003aa1n09x5               g142(.a(\a[22] ), .b(\b[21] ), .c(new_n228), .carry(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  aoi012aa1n02x5               g144(.a(new_n239), .b(new_n222), .c(new_n235), .o1(new_n240));
  xnrc02aa1n12x5               g145(.a(\b[22] ), .b(\a[23] ), .out0(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  xnbna2aa1n03x5               g147(.a(new_n242), .b(new_n237), .c(new_n240), .out0(\s[23] ));
  nor042aa1n06x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  aoi012aa1n03x5               g150(.a(new_n241), .b(new_n237), .c(new_n240), .o1(new_n246));
  tech160nm_fixnrc02aa1n02p5x5 g151(.a(\b[23] ), .b(\a[24] ), .out0(new_n247));
  nano22aa1n03x5               g152(.a(new_n246), .b(new_n245), .c(new_n247), .out0(new_n248));
  inv000aa1n03x5               g153(.a(new_n240), .o1(new_n249));
  aoai13aa1n02x7               g154(.a(new_n242), .b(new_n249), .c(new_n180), .d(new_n236), .o1(new_n250));
  tech160nm_fiaoi012aa1n02p5x5 g155(.a(new_n247), .b(new_n250), .c(new_n245), .o1(new_n251));
  norp02aa1n03x5               g156(.a(new_n251), .b(new_n248), .o1(\s[24] ));
  nor042aa1n06x5               g157(.a(new_n247), .b(new_n241), .o1(new_n253));
  nano22aa1n03x7               g158(.a(new_n217), .b(new_n235), .c(new_n253), .out0(new_n254));
  oai012aa1n06x5               g159(.a(new_n254), .b(new_n194), .c(new_n193), .o1(new_n255));
  inv020aa1n04x5               g160(.a(new_n221), .o1(new_n256));
  aoai13aa1n06x5               g161(.a(new_n235), .b(new_n256), .c(new_n216), .d(new_n212), .o1(new_n257));
  inv040aa1n02x5               g162(.a(new_n253), .o1(new_n258));
  oao003aa1n02x5               g163(.a(\a[24] ), .b(\b[23] ), .c(new_n245), .carry(new_n259));
  aoai13aa1n12x5               g164(.a(new_n259), .b(new_n258), .c(new_n257), .d(new_n238), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  xnrc02aa1n12x5               g166(.a(\b[24] ), .b(\a[25] ), .out0(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n255), .c(new_n261), .out0(\s[25] ));
  nor042aa1n03x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  tech160nm_fiaoi012aa1n05x5   g171(.a(new_n262), .b(new_n255), .c(new_n261), .o1(new_n267));
  tech160nm_fixnrc02aa1n04x5   g172(.a(\b[25] ), .b(\a[26] ), .out0(new_n268));
  nano22aa1n03x7               g173(.a(new_n267), .b(new_n266), .c(new_n268), .out0(new_n269));
  aoai13aa1n03x5               g174(.a(new_n263), .b(new_n260), .c(new_n180), .d(new_n254), .o1(new_n270));
  aoi012aa1n03x5               g175(.a(new_n268), .b(new_n270), .c(new_n266), .o1(new_n271));
  nor002aa1n02x5               g176(.a(new_n271), .b(new_n269), .o1(\s[26] ));
  nor042aa1n09x5               g177(.a(new_n268), .b(new_n262), .o1(new_n273));
  nano32aa1n03x7               g178(.a(new_n217), .b(new_n273), .c(new_n235), .d(new_n253), .out0(new_n274));
  oai012aa1d24x5               g179(.a(new_n274), .b(new_n194), .c(new_n193), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[26] ), .b(\b[25] ), .c(new_n266), .carry(new_n276));
  aobi12aa1n18x5               g181(.a(new_n276), .b(new_n260), .c(new_n273), .out0(new_n277));
  xorc02aa1n12x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  xnbna2aa1n03x5               g183(.a(new_n278), .b(new_n275), .c(new_n277), .out0(\s[27] ));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  inv040aa1n03x5               g185(.a(new_n280), .o1(new_n281));
  aobi12aa1n06x5               g186(.a(new_n278), .b(new_n275), .c(new_n277), .out0(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  nano22aa1n03x7               g188(.a(new_n282), .b(new_n281), .c(new_n283), .out0(new_n284));
  aoai13aa1n06x5               g189(.a(new_n253), .b(new_n239), .c(new_n222), .d(new_n235), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n273), .o1(new_n286));
  aoai13aa1n04x5               g191(.a(new_n276), .b(new_n286), .c(new_n285), .d(new_n259), .o1(new_n287));
  aoai13aa1n02x5               g192(.a(new_n278), .b(new_n287), .c(new_n180), .d(new_n274), .o1(new_n288));
  aoi012aa1n02x5               g193(.a(new_n283), .b(new_n288), .c(new_n281), .o1(new_n289));
  norp02aa1n03x5               g194(.a(new_n289), .b(new_n284), .o1(\s[28] ));
  norb02aa1n02x5               g195(.a(new_n278), .b(new_n283), .out0(new_n291));
  aobi12aa1n06x5               g196(.a(new_n291), .b(new_n275), .c(new_n277), .out0(new_n292));
  oao003aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n281), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  nano22aa1n03x7               g199(.a(new_n292), .b(new_n293), .c(new_n294), .out0(new_n295));
  aoai13aa1n02x5               g200(.a(new_n291), .b(new_n287), .c(new_n180), .d(new_n274), .o1(new_n296));
  aoi012aa1n02x5               g201(.a(new_n294), .b(new_n296), .c(new_n293), .o1(new_n297));
  norp02aa1n03x5               g202(.a(new_n297), .b(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g204(.a(new_n278), .b(new_n294), .c(new_n283), .out0(new_n300));
  aobi12aa1n06x5               g205(.a(new_n300), .b(new_n275), .c(new_n277), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  nano22aa1n03x7               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  aoai13aa1n02x5               g209(.a(new_n300), .b(new_n287), .c(new_n180), .d(new_n274), .o1(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n03x5               g211(.a(new_n306), .b(new_n304), .o1(\s[30] ));
  norb02aa1n03x4               g212(.a(new_n300), .b(new_n303), .out0(new_n308));
  aobi12aa1n06x5               g213(.a(new_n308), .b(new_n275), .c(new_n277), .out0(new_n309));
  oao003aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  nano22aa1n03x7               g216(.a(new_n309), .b(new_n310), .c(new_n311), .out0(new_n312));
  aoai13aa1n02x5               g217(.a(new_n308), .b(new_n287), .c(new_n180), .d(new_n274), .o1(new_n313));
  aoi012aa1n03x5               g218(.a(new_n311), .b(new_n313), .c(new_n310), .o1(new_n314));
  norp02aa1n03x5               g219(.a(new_n314), .b(new_n312), .o1(\s[31] ));
  xnrb03aa1n02x5               g220(.a(new_n117), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g221(.a(\a[3] ), .b(\b[2] ), .c(new_n117), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g223(.a(new_n119), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  orn002aa1n02x5               g224(.a(\a[5] ), .b(\b[4] ), .o(new_n320));
  nanb02aa1n02x5               g225(.a(new_n107), .b(new_n119), .out0(new_n321));
  xobna2aa1n03x5               g226(.a(new_n106), .b(new_n321), .c(new_n320), .out0(\s[6] ));
  norp02aa1n02x5               g227(.a(new_n107), .b(new_n106), .o1(new_n323));
  aobi12aa1n02x5               g228(.a(new_n122), .b(new_n119), .c(new_n323), .out0(new_n324));
  xnrb03aa1n02x5               g229(.a(new_n324), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g230(.a(\a[7] ), .b(\b[6] ), .c(new_n324), .o1(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g232(.a(new_n146), .b(new_n187), .c(new_n189), .out0(\s[9] ));
endmodule


