// Benchmark "adder" written by ABC on Wed Jul 10 17:29:14 2024

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
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n272, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n282, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n301, new_n304, new_n306,
    new_n307, new_n309;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norb02aa1n02x5               g004(.a(new_n99), .b(new_n98), .out0(new_n100));
  norp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  norb02aa1n02x5               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  norp02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  nona23aa1n02x4               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  nano22aa1n02x4               g013(.a(new_n108), .b(new_n100), .c(new_n103), .out0(new_n109));
  nanp02aa1n02x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[0] ), .b(\a[1] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  oai012aa1n02x5               g017(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nona23aa1n02x4               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  aoi012aa1n02x5               g023(.a(new_n114), .b(new_n116), .c(new_n115), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n119), .b(new_n118), .c(new_n113), .o1(new_n120));
  aoi112aa1n02x5               g025(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n121));
  aoi112aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n122));
  oai112aa1n02x5               g027(.a(new_n103), .b(new_n100), .c(new_n122), .d(new_n104), .o1(new_n123));
  nona22aa1n02x4               g028(.a(new_n123), .b(new_n121), .c(new_n98), .out0(new_n124));
  160nm_fiao0012aa1n02p5x5     g029(.a(new_n124), .b(new_n120), .c(new_n109), .o(new_n125));
  nanp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  aoi012aa1n02x5               g031(.a(new_n97), .b(new_n125), .c(new_n126), .o1(new_n127));
  norp02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnrc02aa1n02x5               g035(.a(new_n127), .b(new_n130), .out0(\s[10] ));
  norp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  nano23aa1n02x4               g039(.a(new_n97), .b(new_n128), .c(new_n129), .d(new_n126), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n124), .c(new_n120), .d(new_n109), .o1(new_n136));
  aoi012aa1n02x5               g041(.a(new_n128), .b(new_n97), .c(new_n129), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n134), .b(new_n136), .c(new_n137), .out0(\s[11] ));
  nanp02aa1n02x5               g043(.a(new_n136), .b(new_n137), .o1(new_n139));
  aoi012aa1n02x5               g044(.a(new_n132), .b(new_n139), .c(new_n133), .o1(new_n140));
  xnrb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  and003aa1n02x5               g049(.a(new_n135), .b(new_n134), .c(new_n144), .o(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n124), .c(new_n120), .d(new_n109), .o1(new_n146));
  aoi112aa1n02x5               g051(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n147));
  aoi112aa1n02x5               g052(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n148));
  oai112aa1n02x5               g053(.a(new_n144), .b(new_n134), .c(new_n148), .d(new_n128), .o1(new_n149));
  nona22aa1n02x4               g054(.a(new_n149), .b(new_n147), .c(new_n142), .out0(new_n150));
  160nm_ficinv00aa1n08x5       g055(.clk(new_n150), .clkout(new_n151));
  nanp02aa1n02x5               g056(.a(new_n146), .b(new_n151), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  aoi012aa1n02x5               g060(.a(new_n154), .b(new_n152), .c(new_n155), .o1(new_n156));
  xnrb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nona23aa1n02x4               g064(.a(new_n159), .b(new_n155), .c(new_n154), .d(new_n158), .out0(new_n160));
  aoi012aa1n02x5               g065(.a(new_n158), .b(new_n154), .c(new_n159), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n160), .c(new_n146), .d(new_n151), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  norp02aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(new_n168));
  160nm_ficinv00aa1n08x5       g073(.clk(new_n168), .clkout(new_n169));
  aoi112aa1n02x5               g074(.a(new_n169), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n169), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(\s[16] ));
  nano23aa1n02x4               g077(.a(new_n154), .b(new_n158), .c(new_n159), .d(new_n155), .out0(new_n173));
  nano23aa1n02x4               g078(.a(new_n164), .b(new_n166), .c(new_n167), .d(new_n165), .out0(new_n174));
  nanp02aa1n02x5               g079(.a(new_n174), .b(new_n173), .o1(new_n175));
  nano32aa1n02x4               g080(.a(new_n175), .b(new_n135), .c(new_n144), .d(new_n134), .out0(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n124), .c(new_n120), .d(new_n109), .o1(new_n177));
  nanb02aa1n02x5               g082(.a(new_n164), .b(new_n165), .out0(new_n178));
  norp03aa1n02x5               g083(.a(new_n160), .b(new_n168), .c(new_n178), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n180));
  obai22aa1n02x7               g085(.a(new_n174), .b(new_n161), .c(\a[16] ), .d(\b[15] ), .out0(new_n181));
  aoi112aa1n02x5               g086(.a(new_n181), .b(new_n180), .c(new_n150), .d(new_n179), .o1(new_n182));
  xorc02aa1n02x5               g087(.a(\a[17] ), .b(\b[16] ), .out0(new_n183));
  xnbna2aa1n03x5               g088(.a(new_n183), .b(new_n182), .c(new_n177), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[18] ), .clkout(new_n185));
  nanp02aa1n02x5               g090(.a(new_n182), .b(new_n177), .o1(new_n186));
  norp02aa1n02x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  aoi012aa1n02x5               g092(.a(new_n187), .b(new_n186), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  160nm_ficinv00aa1n08x5       g094(.clk(\a[17] ), .clkout(new_n190));
  xroi22aa1d04x5               g095(.a(new_n190), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(new_n191), .clkout(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(\b[17] ), .clkout(new_n193));
  oao003aa1n02x5               g098(.a(new_n185), .b(new_n193), .c(new_n187), .carry(new_n194));
  160nm_ficinv00aa1n08x5       g099(.clk(new_n194), .clkout(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n192), .c(new_n182), .d(new_n177), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  norp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  aoi112aa1n02x5               g108(.a(new_n199), .b(new_n203), .c(new_n196), .d(new_n200), .o1(new_n204));
  aoai13aa1n02x5               g109(.a(new_n203), .b(new_n199), .c(new_n196), .d(new_n200), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(\s[20] ));
  nano23aa1n02x4               g111(.a(new_n199), .b(new_n201), .c(new_n202), .d(new_n200), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n191), .b(new_n207), .o1(new_n208));
  aoi112aa1n02x5               g113(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n209));
  norp02aa1n02x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  aoi112aa1n02x5               g115(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n200), .b(new_n199), .out0(new_n212));
  oai112aa1n02x5               g117(.a(new_n212), .b(new_n203), .c(new_n211), .d(new_n210), .o1(new_n213));
  nona22aa1n02x4               g118(.a(new_n213), .b(new_n209), .c(new_n201), .out0(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n208), .c(new_n182), .d(new_n177), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  norp02aa1n02x5               g124(.a(\b[21] ), .b(\a[22] ), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(\b[21] ), .b(\a[22] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  aoi112aa1n02x5               g127(.a(new_n218), .b(new_n222), .c(new_n216), .d(new_n219), .o1(new_n223));
  aoai13aa1n02x5               g128(.a(new_n222), .b(new_n218), .c(new_n216), .d(new_n219), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(\s[22] ));
  nona23aa1n02x4               g130(.a(new_n221), .b(new_n219), .c(new_n218), .d(new_n220), .out0(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n226), .clkout(new_n227));
  nanp03aa1n02x5               g132(.a(new_n191), .b(new_n227), .c(new_n207), .o1(new_n228));
  160nm_fiao0012aa1n02p5x5     g133(.a(new_n220), .b(new_n218), .c(new_n221), .o(new_n229));
  aoi012aa1n02x5               g134(.a(new_n229), .b(new_n214), .c(new_n227), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n228), .c(new_n182), .d(new_n177), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(new_n233), .b(new_n235), .c(new_n231), .d(new_n234), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(\s[24] ));
  and002aa1n02x5               g143(.a(new_n235), .b(new_n234), .o(new_n239));
  nanb03aa1n02x5               g144(.a(new_n208), .b(new_n239), .c(new_n227), .out0(new_n240));
  norp02aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .o1(new_n241));
  aoi112aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n242));
  nanp03aa1n02x5               g147(.a(new_n229), .b(new_n234), .c(new_n235), .o1(new_n243));
  nona22aa1n02x4               g148(.a(new_n243), .b(new_n242), .c(new_n241), .out0(new_n244));
  nano22aa1n02x4               g149(.a(new_n226), .b(new_n234), .c(new_n235), .out0(new_n245));
  aoi012aa1n02x5               g150(.a(new_n244), .b(new_n214), .c(new_n245), .o1(new_n246));
  aoai13aa1n02x5               g151(.a(new_n246), .b(new_n240), .c(new_n182), .d(new_n177), .o1(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g153(.a(\b[24] ), .b(\a[25] ), .o1(new_n249));
  xorc02aa1n02x5               g154(.a(\a[25] ), .b(\b[24] ), .out0(new_n250));
  xorc02aa1n02x5               g155(.a(\a[26] ), .b(\b[25] ), .out0(new_n251));
  aoi112aa1n02x5               g156(.a(new_n249), .b(new_n251), .c(new_n247), .d(new_n250), .o1(new_n252));
  aoai13aa1n02x5               g157(.a(new_n251), .b(new_n249), .c(new_n247), .d(new_n250), .o1(new_n253));
  norb02aa1n02x5               g158(.a(new_n253), .b(new_n252), .out0(\s[26] ));
  and002aa1n02x5               g159(.a(new_n251), .b(new_n250), .o(new_n255));
  nano22aa1n02x4               g160(.a(new_n228), .b(new_n255), .c(new_n239), .out0(new_n256));
  norp02aa1n02x5               g161(.a(\b[25] ), .b(\a[26] ), .o1(new_n257));
  aoi112aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n258));
  aoai13aa1n02x5               g163(.a(new_n255), .b(new_n244), .c(new_n214), .d(new_n245), .o1(new_n259));
  nona22aa1n02x4               g164(.a(new_n259), .b(new_n258), .c(new_n257), .out0(new_n260));
  xorc02aa1n02x5               g165(.a(\a[27] ), .b(\b[26] ), .out0(new_n261));
  aoai13aa1n02x5               g166(.a(new_n261), .b(new_n260), .c(new_n186), .d(new_n256), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(new_n260), .b(new_n261), .c(new_n186), .d(new_n256), .o1(new_n263));
  norb02aa1n02x5               g168(.a(new_n262), .b(new_n263), .out0(\s[27] ));
  norp02aa1n02x5               g169(.a(\b[26] ), .b(\a[27] ), .o1(new_n265));
  xorc02aa1n02x5               g170(.a(\a[28] ), .b(\b[27] ), .out0(new_n266));
  nona22aa1n02x4               g171(.a(new_n262), .b(new_n266), .c(new_n265), .out0(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n265), .clkout(new_n268));
  160nm_ficinv00aa1n08x5       g173(.clk(new_n266), .clkout(new_n269));
  aoi012aa1n02x5               g174(.a(new_n269), .b(new_n262), .c(new_n268), .o1(new_n270));
  norb02aa1n02x5               g175(.a(new_n267), .b(new_n270), .out0(\s[28] ));
  and002aa1n02x5               g176(.a(new_n266), .b(new_n261), .o(new_n272));
  aoai13aa1n02x5               g177(.a(new_n272), .b(new_n260), .c(new_n186), .d(new_n256), .o1(new_n273));
  oao003aa1n02x5               g178(.a(\a[28] ), .b(\b[27] ), .c(new_n268), .carry(new_n274));
  xorc02aa1n02x5               g179(.a(\a[29] ), .b(\b[28] ), .out0(new_n275));
  160nm_ficinv00aa1n08x5       g180(.clk(new_n275), .clkout(new_n276));
  aoi012aa1n02x5               g181(.a(new_n276), .b(new_n273), .c(new_n274), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n274), .clkout(new_n278));
  nona22aa1n02x4               g183(.a(new_n273), .b(new_n278), .c(new_n275), .out0(new_n279));
  norb02aa1n02x5               g184(.a(new_n279), .b(new_n277), .out0(\s[29] ));
  xorb03aa1n02x5               g185(.a(new_n111), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g186(.a(new_n276), .b(new_n261), .c(new_n266), .out0(new_n282));
  aoai13aa1n02x5               g187(.a(new_n282), .b(new_n260), .c(new_n186), .d(new_n256), .o1(new_n283));
  oaoi03aa1n02x5               g188(.a(\a[29] ), .b(\b[28] ), .c(new_n274), .o1(new_n284));
  160nm_ficinv00aa1n08x5       g189(.clk(new_n284), .clkout(new_n285));
  xorc02aa1n02x5               g190(.a(\a[30] ), .b(\b[29] ), .out0(new_n286));
  160nm_ficinv00aa1n08x5       g191(.clk(new_n286), .clkout(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(new_n283), .c(new_n285), .o1(new_n288));
  nona22aa1n02x4               g193(.a(new_n283), .b(new_n284), .c(new_n286), .out0(new_n289));
  norb02aa1n02x5               g194(.a(new_n289), .b(new_n288), .out0(\s[30] ));
  nano32aa1n02x4               g195(.a(new_n287), .b(new_n275), .c(new_n266), .d(new_n261), .out0(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n260), .c(new_n186), .d(new_n256), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[30] ), .b(\b[29] ), .c(new_n285), .carry(new_n293));
  160nm_ficinv00aa1n08x5       g198(.clk(new_n293), .clkout(new_n294));
  xorc02aa1n02x5               g199(.a(\a[31] ), .b(\b[30] ), .out0(new_n295));
  nona22aa1n02x4               g200(.a(new_n292), .b(new_n294), .c(new_n295), .out0(new_n296));
  160nm_ficinv00aa1n08x5       g201(.clk(new_n295), .clkout(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n292), .c(new_n293), .o1(new_n298));
  norb02aa1n02x5               g203(.a(new_n296), .b(new_n298), .out0(\s[31] ));
  xnrb03aa1n02x5               g204(.a(new_n113), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g205(.a(\a[3] ), .b(\b[2] ), .c(new_n113), .o1(new_n301));
  xorb03aa1n02x5               g206(.a(new_n301), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g207(.a(new_n120), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g208(.a(new_n106), .b(new_n120), .c(new_n107), .o1(new_n304));
  xnrb03aa1n02x5               g209(.a(new_n304), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g210(.a(new_n108), .b(new_n120), .out0(new_n306));
  nona22aa1n02x4               g211(.a(new_n306), .b(new_n122), .c(new_n104), .out0(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g213(.a(new_n101), .b(new_n307), .c(new_n102), .o1(new_n309));
  xnrc02aa1n02x5               g214(.a(new_n309), .b(new_n100), .out0(\s[8] ));
  xorb03aa1n02x5               g215(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


