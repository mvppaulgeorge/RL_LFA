// Benchmark "adder" written by ABC on Thu Jul 11 12:18:30 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n311, new_n313,
    new_n315;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[10] ), .clkout(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano23aa1n02x4               g011(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n107));
  nanb02aa1n02x5               g012(.a(new_n102), .b(new_n107), .out0(new_n108));
  aoi012aa1n02x5               g013(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n02x4               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nona22aa1n02x4               g021(.a(new_n114), .b(new_n115), .c(new_n116), .out0(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\a[5] ), .clkout(new_n118));
  160nm_ficinv00aa1n08x5       g023(.clk(\b[4] ), .clkout(new_n119));
  nanp02aa1n02x5               g024(.a(new_n119), .b(new_n118), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  oai012aa1n02x5               g026(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n122));
  aobi12aa1n02x5               g027(.a(new_n122), .b(new_n114), .c(new_n121), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n117), .c(new_n108), .d(new_n109), .o1(new_n124));
  xnrc02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  aoib12aa1n02x5               g030(.a(new_n98), .b(new_n124), .c(new_n125), .out0(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  xnrc02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .out0(new_n128));
  norp02aa1n02x5               g033(.a(new_n128), .b(new_n125), .o1(new_n129));
  160nm_ficinv00aa1n08x5       g034(.clk(\b[9] ), .clkout(new_n130));
  oao003aa1n02x5               g035(.a(new_n97), .b(new_n130), .c(new_n98), .carry(new_n131));
  aoi012aa1n02x5               g036(.a(new_n131), .b(new_n124), .c(new_n129), .o1(new_n132));
  xnrb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g038(.a(\a[11] ), .b(\b[10] ), .c(new_n132), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g040(.a(\b[12] ), .b(\a[13] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[12] ), .b(\a[13] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  norp02aa1n02x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nano23aa1n02x4               g047(.a(new_n139), .b(new_n141), .c(new_n142), .d(new_n140), .out0(new_n143));
  oai012aa1n02x5               g048(.a(new_n142), .b(new_n141), .c(new_n139), .o1(new_n144));
  aobi12aa1n02x5               g049(.a(new_n144), .b(new_n143), .c(new_n131), .out0(new_n145));
  nona23aa1n02x4               g050(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n146));
  oai012aa1n02x5               g051(.a(new_n109), .b(new_n146), .c(new_n102), .o1(new_n147));
  nona23aa1n02x4               g052(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n148));
  norp03aa1n02x5               g053(.a(new_n148), .b(new_n115), .c(new_n116), .o1(new_n149));
  oaib12aa1n02x5               g054(.a(new_n122), .b(new_n148), .c(new_n121), .out0(new_n150));
  nona23aa1n02x4               g055(.a(new_n142), .b(new_n140), .c(new_n139), .d(new_n141), .out0(new_n151));
  norp03aa1n02x5               g056(.a(new_n151), .b(new_n128), .c(new_n125), .o1(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n150), .c(new_n147), .d(new_n149), .o1(new_n153));
  xnbna2aa1n03x5               g058(.a(new_n138), .b(new_n153), .c(new_n145), .out0(\s[13] ));
  oaoi03aa1n02x5               g059(.a(new_n97), .b(new_n130), .c(new_n98), .o1(new_n155));
  oai012aa1n02x5               g060(.a(new_n144), .b(new_n151), .c(new_n155), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n138), .b(new_n156), .c(new_n124), .d(new_n152), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n136), .out0(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nano23aa1n02x4               g066(.a(new_n136), .b(new_n160), .c(new_n161), .d(new_n137), .out0(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(new_n162), .clkout(new_n163));
  oai012aa1n02x5               g068(.a(new_n161), .b(new_n160), .c(new_n136), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n153), .d(new_n145), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  xorc02aa1n02x5               g072(.a(\a[15] ), .b(\b[14] ), .out0(new_n168));
  xorc02aa1n02x5               g073(.a(\a[16] ), .b(\b[15] ), .out0(new_n169));
  aoi112aa1n02x5               g074(.a(new_n169), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n169), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(\s[16] ));
  nanp03aa1n02x5               g077(.a(new_n162), .b(new_n168), .c(new_n169), .o1(new_n173));
  nano22aa1n02x4               g078(.a(new_n173), .b(new_n129), .c(new_n143), .out0(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n150), .c(new_n147), .d(new_n149), .o1(new_n175));
  xnrc02aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .out0(new_n176));
  xnrc02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .out0(new_n177));
  160nm_ficinv00aa1n08x5       g082(.clk(new_n167), .clkout(new_n178));
  oao003aa1n02x5               g083(.a(\a[16] ), .b(\b[15] ), .c(new_n178), .carry(new_n179));
  oai013aa1n02x4               g084(.a(new_n179), .b(new_n177), .c(new_n176), .d(new_n164), .o1(new_n180));
  aoib12aa1n02x5               g085(.a(new_n180), .b(new_n156), .c(new_n173), .out0(new_n181));
  nanp02aa1n02x5               g086(.a(new_n175), .b(new_n181), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g088(.clk(\a[18] ), .clkout(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[17] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\b[16] ), .clkout(new_n186));
  oaoi03aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n182), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n184), .out0(\s[18] ));
  xroi22aa1d04x5               g093(.a(new_n185), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n189));
  oai022aa1n02x5               g094(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n190));
  oaib12aa1n02x5               g095(.a(new_n190), .b(new_n184), .c(\b[17] ), .out0(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(new_n191), .clkout(new_n192));
  norp02aa1n02x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n192), .c(new_n182), .d(new_n189), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n195), .b(new_n192), .c(new_n182), .d(new_n189), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  nona22aa1n02x4               g107(.a(new_n196), .b(new_n202), .c(new_n193), .out0(new_n203));
  160nm_ficinv00aa1n08x5       g108(.clk(new_n193), .clkout(new_n204));
  aobi12aa1n02x5               g109(.a(new_n202), .b(new_n196), .c(new_n204), .out0(new_n205));
  norb02aa1n02x5               g110(.a(new_n203), .b(new_n205), .out0(\s[20] ));
  nano23aa1n02x4               g111(.a(new_n193), .b(new_n200), .c(new_n201), .d(new_n194), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n189), .b(new_n207), .o1(new_n208));
  nona23aa1n02x4               g113(.a(new_n201), .b(new_n194), .c(new_n193), .d(new_n200), .out0(new_n209));
  oaoi03aa1n02x5               g114(.a(\a[20] ), .b(\b[19] ), .c(new_n204), .o1(new_n210));
  160nm_ficinv00aa1n08x5       g115(.clk(new_n210), .clkout(new_n211));
  oai012aa1n02x5               g116(.a(new_n211), .b(new_n209), .c(new_n191), .o1(new_n212));
  160nm_ficinv00aa1n08x5       g117(.clk(new_n212), .clkout(new_n213));
  aoai13aa1n02x5               g118(.a(new_n213), .b(new_n208), .c(new_n175), .d(new_n181), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xorc02aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  aoi112aa1n02x5               g123(.a(new_n216), .b(new_n218), .c(new_n214), .d(new_n217), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n218), .b(new_n216), .c(new_n214), .d(new_n217), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g126(.clk(\a[21] ), .clkout(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(\a[22] ), .clkout(new_n223));
  xroi22aa1d04x5               g128(.a(new_n222), .b(\b[20] ), .c(new_n223), .d(\b[21] ), .out0(new_n224));
  nanp03aa1n02x5               g129(.a(new_n224), .b(new_n189), .c(new_n207), .o1(new_n225));
  160nm_ficinv00aa1n08x5       g130(.clk(\b[21] ), .clkout(new_n226));
  oao003aa1n02x5               g131(.a(new_n223), .b(new_n226), .c(new_n216), .carry(new_n227));
  aoi012aa1n02x5               g132(.a(new_n227), .b(new_n212), .c(new_n224), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n228), .b(new_n225), .c(new_n175), .d(new_n181), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g135(.a(\b[22] ), .b(\a[23] ), .o1(new_n231));
  xorc02aa1n02x5               g136(.a(\a[23] ), .b(\b[22] ), .out0(new_n232));
  xorc02aa1n02x5               g137(.a(\a[24] ), .b(\b[23] ), .out0(new_n233));
  aoi112aa1n02x5               g138(.a(new_n231), .b(new_n233), .c(new_n229), .d(new_n232), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n233), .b(new_n231), .c(new_n229), .d(new_n232), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(\s[24] ));
  oabi12aa1n02x5               g141(.a(new_n180), .b(new_n145), .c(new_n173), .out0(new_n237));
  and002aa1n02x5               g142(.a(new_n233), .b(new_n232), .o(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(new_n238), .clkout(new_n239));
  nano32aa1n02x4               g144(.a(new_n239), .b(new_n224), .c(new_n189), .d(new_n207), .out0(new_n240));
  aoai13aa1n02x5               g145(.a(new_n240), .b(new_n237), .c(new_n124), .d(new_n174), .o1(new_n241));
  aoai13aa1n02x5               g146(.a(new_n224), .b(new_n210), .c(new_n207), .d(new_n192), .o1(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n227), .clkout(new_n243));
  oai022aa1n02x5               g148(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n244));
  aob012aa1n02x5               g149(.a(new_n244), .b(\b[23] ), .c(\a[24] ), .out0(new_n245));
  aoai13aa1n02x5               g150(.a(new_n245), .b(new_n239), .c(new_n242), .d(new_n243), .o1(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n246), .clkout(new_n247));
  xnrc02aa1n02x5               g152(.a(\b[24] ), .b(\a[25] ), .out0(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(new_n248), .clkout(new_n249));
  xnbna2aa1n03x5               g154(.a(new_n249), .b(new_n241), .c(new_n247), .out0(\s[25] ));
  norp02aa1n02x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  160nm_ficinv00aa1n08x5       g156(.clk(new_n251), .clkout(new_n252));
  aoai13aa1n02x5               g157(.a(new_n249), .b(new_n246), .c(new_n182), .d(new_n240), .o1(new_n253));
  xnrc02aa1n02x5               g158(.a(\b[25] ), .b(\a[26] ), .out0(new_n254));
  nanp03aa1n02x5               g159(.a(new_n253), .b(new_n252), .c(new_n254), .o1(new_n255));
  aoi012aa1n02x5               g160(.a(new_n254), .b(new_n253), .c(new_n252), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n255), .b(new_n256), .out0(\s[26] ));
  norp02aa1n02x5               g162(.a(new_n254), .b(new_n248), .o1(new_n258));
  nano22aa1n02x4               g163(.a(new_n225), .b(new_n238), .c(new_n258), .out0(new_n259));
  aoai13aa1n02x5               g164(.a(new_n259), .b(new_n237), .c(new_n124), .d(new_n174), .o1(new_n260));
  nanp02aa1n02x5               g165(.a(new_n246), .b(new_n258), .o1(new_n261));
  oao003aa1n02x5               g166(.a(\a[26] ), .b(\b[25] ), .c(new_n252), .carry(new_n262));
  xorc02aa1n02x5               g167(.a(\a[27] ), .b(\b[26] ), .out0(new_n263));
  160nm_ficinv00aa1n08x5       g168(.clk(new_n263), .clkout(new_n264));
  aoi013aa1n02x4               g169(.a(new_n264), .b(new_n260), .c(new_n261), .d(new_n262), .o1(new_n265));
  aobi12aa1n02x5               g170(.a(new_n259), .b(new_n175), .c(new_n181), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n238), .b(new_n227), .c(new_n212), .d(new_n224), .o1(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n258), .clkout(new_n268));
  aoai13aa1n02x5               g173(.a(new_n262), .b(new_n268), .c(new_n267), .d(new_n245), .o1(new_n269));
  norp03aa1n02x5               g174(.a(new_n269), .b(new_n266), .c(new_n263), .o1(new_n270));
  norp02aa1n02x5               g175(.a(new_n265), .b(new_n270), .o1(\s[27] ));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(new_n272), .clkout(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[27] ), .b(\a[28] ), .out0(new_n274));
  nano22aa1n02x4               g179(.a(new_n265), .b(new_n273), .c(new_n274), .out0(new_n275));
  oai012aa1n02x5               g180(.a(new_n263), .b(new_n269), .c(new_n266), .o1(new_n276));
  aoi012aa1n02x5               g181(.a(new_n274), .b(new_n276), .c(new_n273), .o1(new_n277));
  norp02aa1n02x5               g182(.a(new_n277), .b(new_n275), .o1(\s[28] ));
  norb02aa1n02x5               g183(.a(new_n263), .b(new_n274), .out0(new_n279));
  oai012aa1n02x5               g184(.a(new_n279), .b(new_n269), .c(new_n266), .o1(new_n280));
  oao003aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .c(new_n273), .carry(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  aoi012aa1n02x5               g187(.a(new_n282), .b(new_n280), .c(new_n281), .o1(new_n283));
  160nm_ficinv00aa1n08x5       g188(.clk(new_n279), .clkout(new_n284));
  aoi013aa1n02x4               g189(.a(new_n284), .b(new_n260), .c(new_n261), .d(new_n262), .o1(new_n285));
  nano22aa1n02x4               g190(.a(new_n285), .b(new_n281), .c(new_n282), .out0(new_n286));
  norp02aa1n02x5               g191(.a(new_n283), .b(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g192(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g193(.a(new_n263), .b(new_n282), .c(new_n274), .out0(new_n289));
  oai012aa1n02x5               g194(.a(new_n289), .b(new_n269), .c(new_n266), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n281), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[29] ), .b(\a[30] ), .out0(new_n292));
  aoi012aa1n02x5               g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n293));
  160nm_ficinv00aa1n08x5       g198(.clk(new_n289), .clkout(new_n294));
  aoi013aa1n02x4               g199(.a(new_n294), .b(new_n260), .c(new_n261), .d(new_n262), .o1(new_n295));
  nano22aa1n02x4               g200(.a(new_n295), .b(new_n291), .c(new_n292), .out0(new_n296));
  norp02aa1n02x5               g201(.a(new_n293), .b(new_n296), .o1(\s[30] ));
  norb02aa1n02x5               g202(.a(new_n289), .b(new_n292), .out0(new_n298));
  160nm_ficinv00aa1n08x5       g203(.clk(new_n298), .clkout(new_n299));
  aoi013aa1n02x4               g204(.a(new_n299), .b(new_n260), .c(new_n261), .d(new_n262), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .c(new_n291), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n300), .b(new_n301), .c(new_n302), .out0(new_n303));
  oai012aa1n02x5               g208(.a(new_n298), .b(new_n269), .c(new_n266), .o1(new_n304));
  aoi012aa1n02x5               g209(.a(new_n302), .b(new_n304), .c(new_n301), .o1(new_n305));
  norp02aa1n02x5               g210(.a(new_n305), .b(new_n303), .o1(\s[31] ));
  xnrb03aa1n02x5               g211(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g212(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g214(.a(new_n147), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g215(.a(new_n118), .b(new_n119), .c(new_n147), .o1(new_n311));
  xnrb03aa1n02x5               g216(.a(new_n311), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g217(.a(\a[6] ), .b(\b[5] ), .c(new_n311), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g219(.a(new_n112), .b(new_n313), .c(new_n113), .o1(new_n315));
  xnrb03aa1n02x5               g220(.a(new_n315), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g221(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


